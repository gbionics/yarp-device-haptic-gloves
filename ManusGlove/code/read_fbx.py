"""
Read a binary FBX file (e.g. a MANUS Core FBX export) and print what it contains.

Sections (all printed by default, or select them with the flags below):
  * --info       file header, axis system, units, frame rate, object counts, mesh
  * --hierarchy  bone tree with the static local transform of every bone
  * --poses      bind/rest poses stored in the file
  * --animation  animation stacks and, for every animated bone, its channels
                 (number of keys and value range)

--csv FILE exports the animated local transforms of the bones, sampled at the
file frame rate, one row per frame (rotations are Euler angles in degrees, in the
bone RotationOrder; translations in file units). Curves are sampled with linear
interpolation; channels without a curve keep their static value.

Notes on MANUS Core exports (https://docs.manus-meta.com/3.2.0/Software/Rec/Exporting%20fbx/):
  * the skeleton is the retargeted one (the target skeleton chosen in the export
    settings), not the MANUS raw skeleton;
  * bind pose, rest pose and static bone values are the skeleton reference pose;
    if "Use first frame bindpose" was enabled, animation frame 0 is that pose too
    and not recorded data;
  * animation rotations are absolute local rotations, not offsets from the rest pose.

Only binary FBX 7.x is supported (no external dependencies).

Usage:
    python read_fbx.py recording.fbx
    python read_fbx.py recording.fbx --hierarchy --bones "_r$"
    python read_fbx.py recording.fbx --csv recording_anim.csv --bones "^(thumb|index|middle|ring|pinky)_"
"""
import argparse
import collections
import csv
import re
import struct
import sys
import zlib

FBX_TICKS_PER_SECOND = 46186158000

# FbxTime::EMode -> frames per second (None = custom, see CustomFrameRate)
TIME_MODES = {0: None, 1: 120.0, 2: 100.0, 3: 60.0, 4: 50.0, 5: 48.0, 6: 30.0, 7: 30.0, 8: 29.97, 9: 29.97,
              10: 25.0, 11: 24.0, 12: 1000.0, 13: 23.976, 14: None, 15: 96.0, 16: 72.0, 17: 59.94, 18: 119.88}
ROTATION_ORDERS = {0: "XYZ", 1: "XZY", 2: "YZX", 3: "YXZ", 4: "ZXY", 5: "ZYX", 6: "SphericXYZ"}
AXES = "XYZ"


# ---------------------------------------------------------------------------
# Binary FBX parsing
# ---------------------------------------------------------------------------
class Node:
    __slots__ = ("name", "props", "children")

    def __init__(self, name, props, children):
        self.name, self.props, self.children = name, props, children

    def find(self, name):
        return next((c for c in self.children if c.name == name), None)

    def findall(self, name):
        return [c for c in self.children if c.name == name]


_ARRAY_FMT = {"f": "f", "d": "d", "l": "q", "i": "i", "b": "?"}
_SCALAR_FMT = {"Y": "<h", "C": "<?", "I": "<i", "F": "<f", "D": "<d", "L": "<q"}


def _read_property(buf, pos):
    kind = chr(buf[pos])
    pos += 1
    if kind in _SCALAR_FMT:
        fmt = _SCALAR_FMT[kind]
        return struct.unpack_from(fmt, buf, pos)[0], pos + struct.calcsize(fmt)
    if kind in "SR":
        size = struct.unpack_from("<I", buf, pos)[0]
        data = bytes(buf[pos + 4:pos + 4 + size])
        return (data.decode("utf-8", "replace") if kind == "S" else data), pos + 4 + size
    if kind in _ARRAY_FMT:
        length, encoding, size = struct.unpack_from("<III", buf, pos)
        data = bytes(buf[pos + 12:pos + 12 + size])
        if encoding == 1:
            data = zlib.decompress(data)
        return list(struct.unpack(f"<{length}{_ARRAY_FMT[kind]}", data)), pos + 12 + size
    raise ValueError(f"Unknown FBX property type {kind!r} at byte {pos - 1}")


def _read_node(buf, pos, wide):
    header = "<QQQ" if wide else "<III"
    end, n_props, _ = struct.unpack_from(header, buf, pos)
    pos += struct.calcsize(header)
    name_len = buf[pos]
    pos += 1
    if end == 0:
        return None, pos
    name = bytes(buf[pos:pos + name_len]).decode("ascii", "replace")
    pos += name_len
    props = []
    for _ in range(n_props):
        value, pos = _read_property(buf, pos)
        props.append(value)
    children = []
    while pos < end:
        child, pos = _read_node(buf, pos, wide)
        if child is None:
            break
        children.append(child)
    return Node(name, props, children), end


def load_fbx(path):
    with open(path, "rb") as f:
        buf = f.read()
    if not buf.startswith(b"Kaydara FBX Binary"):
        sys.exit(f"{path}: not a binary FBX file (ASCII FBX is not supported).")
    version = struct.unpack_from("<I", buf, 23)[0]
    wide = version >= 7500
    pos, top = 27, []
    while pos < len(buf):
        node, pos = _read_node(buf, pos, wide)
        if node is None:
            break
        top.append(node)
    return version, Node("root", [], top)


def properties70(node):
    """Properties70 block -> {name: [values]}."""
    block = node.find("Properties70") if node is not None else None
    return {p.props[0]: p.props[4:] for p in (block.children if block else [])}


# ---------------------------------------------------------------------------
# Scene model
# ---------------------------------------------------------------------------
class Scene:
    def __init__(self, path):
        self.version, self.root = load_fbx(path)
        objects = self.root.find("Objects")
        self.objects = {o.props[0]: o for o in (objects.children if objects else [])}
        self.connections = [c.props for c in (self.root.find("Connections") or Node("", [], [])).children]
        self.settings = properties70(self.root.find("GlobalSettings"))

        self.models = {k: o for k, o in self.objects.items() if o.name == "Model"}
        self.parent = {}
        self.curve_nodes = {}                      # curve node id -> (model id, property)
        self.curves = collections.defaultdict(dict)  # curve node id -> {channel: curve id}
        for kind, src, dst, *rest in self.connections:
            # A bone can also be connected to skin deformers: only Model -> Model links define the hierarchy.
            if kind == "OO" and src in self.models and dst in self.models:
                self.parent[src] = dst
            src_obj = self.objects.get(src)
            if kind == "OP" and src_obj is not None and rest:
                if src_obj.name == "AnimationCurveNode" and dst in self.models:
                    self.curve_nodes[src] = (dst, rest[0])
                elif src_obj.name == "AnimationCurve":
                    self.curves[dst][rest[0]] = src
        self.children = collections.defaultdict(list)
        for child, parent in self.parent.items():
            self.children[parent].append(child)

    @staticmethod
    def name(obj):
        return obj.props[1].split("\x00")[0]

    def model_type(self, key):
        return self.models[key].props[2]

    def roots(self):
        return [k for k in self.models if k not in self.parent]

    def frame_rate(self):
        mode = self.settings.get("TimeMode", [None])[0]
        fps = TIME_MODES.get(mode)
        if fps is None:
            custom = self.settings.get("CustomFrameRate", [-1.0])[0]
            fps = custom if custom and custom > 0 else None
        return fps

    def static_transform(self, key):
        props = properties70(self.models[key])
        return {
            "T": list(props.get("Lcl Translation", [0.0, 0.0, 0.0])),
            "R": list(props.get("Lcl Rotation", [0.0, 0.0, 0.0])),
            "S": list(props.get("Lcl Scaling", [1.0, 1.0, 1.0])),
            "order": ROTATION_ORDERS.get(props.get("RotationOrder", [0])[0], "?"),
            "pre": props.get("PreRotation"),
            "post": props.get("PostRotation"),
        }

    def animated_channels(self, key):
        """{('T'|'R'|'S', axis): (times_s, values)} for the curves driving this model."""
        out = {}
        for cn, (model, prop) in self.curve_nodes.items():
            if model != key or not prop.startswith("Lcl "):
                continue
            letter = {"Lcl Translation": "T", "Lcl Rotation": "R", "Lcl Scaling": "S"}.get(prop)
            for channel, curve_id in self.curves.get(cn, {}).items():
                curve = self.objects[curve_id]
                times = [t / FBX_TICKS_PER_SECOND for t in curve.find("KeyTime").props[0]]
                values = curve.find("KeyValueFloat").props[0]
                out[(letter, channel[-1])] = (times, values)
        return out

    def time_span(self):
        stacks = [o for o in self.objects.values() if o.name == "AnimationStack"]
        if stacks:
            p = properties70(stacks[0])
            start = p.get("LocalStart", [0])[0] / FBX_TICKS_PER_SECOND
            stop = p.get("LocalStop", p.get("ReferenceStop", [0]))[0] / FBX_TICKS_PER_SECOND
            return start, stop
        return (self.settings.get("TimeSpanStart", [0])[0] / FBX_TICKS_PER_SECOND,
                self.settings.get("TimeSpanStop", [0])[0] / FBX_TICKS_PER_SECOND)


def interp(times, values, t):
    if t <= times[0]:
        return values[0]
    if t >= times[-1]:
        return values[-1]
    lo, hi = 0, len(times) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if times[mid] <= t:
            lo = mid
        else:
            hi = mid
    a = (t - times[lo]) / (times[hi] - times[lo])
    return values[lo] + a * (values[hi] - values[lo])


def fmt(values, digits=2):
    return "(" + ", ".join(f"{v:.{digits}f}" for v in values) + ")"


# ---------------------------------------------------------------------------
# Report sections
# ---------------------------------------------------------------------------
def print_info(scene, path):
    print(f"=== File: {path}")
    print(f"FBX version: {scene.version}")
    creator = scene.root.find("Creator")
    if creator is not None:
        print(f"Creator: {creator.props[0]}")
    header = scene.root.find("FBXHeaderExtension")
    stamp = header.find("CreationTimeStamp") if header else None
    if stamp is not None:
        v = {c.name: c.props[0] for c in stamp.children}
        print(f"Created: {v.get('Year')}-{v.get('Month'):02d}-{v.get('Day'):02d} "
              f"{v.get('Hour'):02d}:{v.get('Minute'):02d}:{v.get('Second'):02d}")

    s = scene.settings
    axis = {}
    for role in ("Up", "Front", "Coord"):
        idx, sign = s.get(f"{role}Axis", [None])[0], s.get(f"{role}AxisSign", [1])[0]
        if idx is not None:
            vec = [0, 0, 0]
            vec[idx] = sign
            axis[role] = vec
            print(f"{role} axis: {'+' if sign > 0 else '-'}{AXES[idx]}")
    if len(axis) == 3:
        c, u, f = axis["Coord"], axis["Up"], axis["Front"]
        cross = [c[1] * u[2] - c[2] * u[1], c[2] * u[0] - c[0] * u[2], c[0] * u[1] - c[1] * u[0]]
        print(f"Handedness: {'right' if sum(a * b for a, b in zip(cross, f)) > 0 else 'left'}-handed")
    unit = s.get("UnitScaleFactor", [1.0])[0]
    print(f"Unit: 1 unit = {unit:g} cm")
    fps = scene.frame_rate()
    start, stop = scene.time_span()
    frames = f", {round((stop - start) * fps) + 1} frames" if fps else ""
    print(f"Frame rate: {fps if fps else 'unknown'} fps; time span {start:.3f}-{stop:.3f} s{frames}")

    counts = collections.Counter(
        (o.name, o.props[2] if len(o.props) > 2 and isinstance(o.props[2], str) and o.props[2] else "")
        for o in scene.objects.values())
    print("Objects:")
    for (kind, sub), n in sorted(counts.items()):
        print(f"  {kind}{' / ' + sub if sub else ''}: {n}")

    for o in scene.objects.values():
        if o.name == "Geometry" and o.find("Vertices") is not None:
            n_vert = len(o.find("Vertices").props[0]) // 3
            poly = o.find("PolygonVertexIndex")
            n_poly = sum(1 for i in poly.props[0] if i < 0) if poly is not None else 0
            print(f"Mesh '{scene.name(o)}': {n_vert} vertices, {n_poly} polygons")
        if o.name == "Constraint":
            print(f"Constraint '{scene.name(o)}' ({o.props[2]})")


def print_hierarchy(scene, bone_filter):
    print("=== Hierarchy (static local transform: T in file units, R in degrees)")

    def walk(key, depth):
        t = scene.static_transform(key)
        name = scene.name(scene.models[key])
        shown = bone_filter is None or bone_filter.search(name)
        if shown:
            extra = f" order={t['order']}"
            if t["pre"]:
                extra += f" pre={fmt(t['pre'])}"
            if t["post"]:
                extra += f" post={fmt(t['post'])}"
            print(f"{'  ' * depth}{name} [{scene.model_type(key)}] T={fmt(t['T'])} R={fmt(t['R'])}{extra}")
        for child in scene.children.get(key, []):
            walk(child, depth + 1 if shown else depth)

    for root in scene.roots():
        walk(root, 0)


def print_poses(scene):
    print("=== Poses")
    for o in scene.objects.values():
        if o.name != "Pose":
            continue
        nodes = o.findall("PoseNode")
        names = []
        for pn in nodes:
            target = scene.objects.get(pn.find("Node").props[0])
            names.append(scene.name(target) if target is not None else "<scene root>")
        print(f"{o.props[2]} '{scene.name(o)}': {len(nodes)} nodes")
        print(f"  {', '.join(names)}")
    print("  (MANUS exports: BindPose has global matrices, RestPose local ones; both are the skeleton reference pose)")


def print_animation(scene, bone_filter):
    print("=== Animation")
    for o in scene.objects.values():
        if o.name == "AnimationStack":
            start, stop = scene.time_span()
            print(f"Stack '{scene.name(o)}': {start:.3f}-{stop:.3f} s")
    for key in sorted(scene.models, key=lambda k: scene.name(scene.models[k])):
        name = scene.name(scene.models[key])
        if bone_filter is not None and not bone_filter.search(name):
            continue
        channels = scene.animated_channels(key)
        if not channels:
            continue
        parts = [f"{p}{a}: {len(v)} keys [{min(v):.2f}, {max(v):.2f}]" for (p, a), (_, v) in sorted(channels.items())]
        print(f"  {name}: " + "; ".join(parts))


def export_csv(scene, path, bone_filter):
    fps = scene.frame_rate()
    if not fps:
        sys.exit("Cannot export CSV: the file frame rate is unknown.")
    start, stop = scene.time_span()
    n_frames = round((stop - start) * fps) + 1
    bones = []
    for key in sorted(scene.models, key=lambda k: scene.name(scene.models[k])):
        name = scene.name(scene.models[key])
        if scene.model_type(key) != "LimbNode" or (bone_filter is not None and not bone_filter.search(name)):
            continue
        channels = scene.animated_channels(key)
        if channels:
            bones.append((name, scene.static_transform(key), channels))
    if not bones:
        sys.exit("No animated bones match the selection.")

    columns = [(name, p, a) for name, _, ch in bones for p in "TRS" for a in AXES
               if any(k[0] == p for k in ch)]
    labels = {"T": "Translation", "R": "Rotation", "S": "Scaling"}
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["Frame", "Time_s"] + [f"{n}_{labels[p]}_{a}" for n, p, a in columns])
        lookup = {name: (static, ch) for name, static, ch in bones}
        for i in range(n_frames):
            t = start + i / fps
            row = [i, f"{t:.6f}"]
            for name, p, a in columns:
                static, ch = lookup[name]
                if (p, a) in ch:
                    row.append(f"{interp(*ch[(p, a)], t):.6f}")
                else:
                    row.append(f"{static[p][AXES.index(a)]:.6f}")
            writer.writerow(row)
    print(f"Wrote {path}: {n_frames} frames x {len(columns)} channels of {len(bones)} bones "
          f"(frame 0 is the bind pose if 'Use first frame bindpose' was enabled in the export).")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("fbx", help="Binary FBX file")
    parser.add_argument("--info", action="store_true", help="Print file info")
    parser.add_argument("--hierarchy", action="store_true", help="Print the bone hierarchy")
    parser.add_argument("--poses", action="store_true", help="Print bind/rest poses")
    parser.add_argument("--animation", action="store_true", help="Print animated channels")
    parser.add_argument("--bones", help="Regular expression to select bones (hierarchy, animation, csv)")
    parser.add_argument("--csv", help="Export the animated bone transforms to this CSV file")
    args = parser.parse_args()

    scene = Scene(args.fbx)
    bone_filter = re.compile(args.bones) if args.bones else None
    show_all = not (args.info or args.hierarchy or args.poses or args.animation or args.csv)

    if show_all or args.info:
        print_info(scene, args.fbx)
    if show_all or args.hierarchy:
        print_hierarchy(scene, bone_filter)
    if show_all or args.poses:
        print_poses(scene)
    if show_all or args.animation:
        print_animation(scene, bone_filter)
    if args.csv:
        export_csv(scene, args.csv, bone_filter)


if __name__ == "__main__":
    main()
