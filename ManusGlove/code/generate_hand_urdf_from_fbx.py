"""
Generate one URDF per hand from the reference pose (bind pose) of a MANUS Core FBX export.

In the FBX export MANUS Core applies the recorded data to a target skeleton chosen
in the export settings. This script supports the "MANUS Polygon Avatar" skeleton
(the only one that can include the hand meshes). With "Scale skeleton" enabled
the skeleton is scaled to the user; its reference pose (static bone values = bind
pose = rest pose, and also animation frame 0 if "Use first frame bindpose" is
enabled) does not depend on the pose the user had while recording. This script
turns that pose into a URDF, so that joint angles equal to zero reproduce it.

The FBX data are used as they are (no change to the pose). Links, joints, axes,
sign conventions and limits are the same as in generate_hand_urdf.py (the FBX bone
axes are only relabeled to the MANUS SDK convention: bone along +X, flexion about +Y),
so the two URDFs are interchangeable; only the geometry source differs:
  * phalanx lengths: the user's (MANUS calibration), only if "Scale skeleton" was enabled;
  * metacarpals, palm and pose (finger bends, spread, thumb orientation): the target skeleton.
See https://docs.manus-meta.com/3.2.0/Software/Rec/Exporting%20fbx/ for the export settings.

FBX bone -> URDF link (same names as generate_hand_urdf.py, <s> = l or r):
    hand_<side>                -> <s>_hand
    <finger>_metacarpal_<side> -> <s>_<finger>_metacarpal   (fixed)
    <finger>_01/_02/_03_<side> -> <s>_<finger>_proximal / _intermediate / _distal
    thumb_01/_02/_03_<side>    -> <s>_thumb_metacarpal / _proximal / _distal
    <finger>_tip_<side>        -> <s>_<finger>_tip         (fixed)

Usage:
    python generate_hand_urdf_from_fbx.py recording.fbx
    python generate_hand_urdf_from_fbx.py recording.fbx --side right --out-dir <OUTPUT_DIR>
"""
import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from generate_hand_urdf import build_hand_urdf  # noqa: E402
from read_fbx import Scene, properties70, ROTATION_ORDERS  # noqa: E402

FINGERS = ("thumb", "index", "middle", "ring", "pinky")
# FBX bone suffix -> MANUS FingerJointType
FINGER_SEGMENTS = (("metacarpal", "Metacarpal"), ("01", "Proximal"), ("02", "Intermediate"),
                   ("03", "Distal"), ("tip", "Tip"))
THUMB_SEGMENTS = (("01", "Metacarpal"), ("02", "Proximal"), ("03", "Distal"), ("tip", "Tip"))
# Rows: SDK node axes (bone +X, flexion hinge +Y, Z = X x Y) expressed in the FBX bone axes.
FBX_TO_SDK_AXES = {
    "l": [[1, 0, 0], [0, 0, 1], [0, -1, 0]],
    "r": [[1, 0, 0], [0, 0, -1], [0, 1, 0]],
}


# ---------------------------------------------------------------------------
# Small 3x3 / 4x4 matrix helpers (lists, no dependencies)
# ---------------------------------------------------------------------------
def matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(len(b))) for j in range(len(b[0]))] for i in range(len(a))]


def axis_rotation(axis, deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    if axis == "X":
        return [[1, 0, 0], [0, c, -s], [0, s, c]]
    if axis == "Y":
        return [[c, 0, s], [0, 1, 0], [-s, 0, c]]
    return [[c, -s, 0], [s, c, 0], [0, 0, 1]]


def euler_to_matrix(angles_deg, order="XYZ"):
    """FBX Euler angles: the first axis of `order` is applied first (XYZ -> Rz * Ry * Rx)."""
    m = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    for axis in order:
        m = matmul(axis_rotation(axis, angles_deg["XYZ".index(axis)]), m)
    return m


def transpose(m):
    return [list(r) for r in zip(*m)]


def homogeneous(rot, trans):
    return [rot[0] + [trans[0]], rot[1] + [trans[1]], rot[2] + [trans[2]], [0, 0, 0, 1]]


def matrix_to_quat(m):
    """Rotation matrix -> (w, x, y, z)."""
    tr = m[0][0] + m[1][1] + m[2][2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        q = (0.25 * s, (m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s)
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2
        q = ((m[2][1] - m[1][2]) / s, 0.25 * s, (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s)
    elif m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2
        q = ((m[0][2] - m[2][0]) / s, (m[0][1] + m[1][0]) / s, 0.25 * s, (m[1][2] + m[2][1]) / s)
    else:
        s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2
        q = ((m[1][0] - m[0][1]) / s, (m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, 0.25 * s)
    n = math.sqrt(sum(c * c for c in q))
    return tuple(c / n for c in q)


def rotation_angle_deg(a, b):
    """Angle of the rotation between two 3x3 rotation matrices."""
    c = (sum(a[i][k] * b[i][k] for i in range(3) for k in range(3)) - 1) / 2
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


# ---------------------------------------------------------------------------
# FBX rest pose -> MANUS-style nodes
# ---------------------------------------------------------------------------
def local_rest_transform(scene, key, warnings):
    """Static local transform of a bone (= rest pose in MANUS exports), as a 4x4 matrix in file units."""
    props = properties70(scene.models[key])
    name = scene.name(scene.models[key])
    order = ROTATION_ORDERS.get(props.get("RotationOrder", [0])[0], "XYZ")
    if order not in ("XYZ", "XZY", "YZX", "YXZ", "ZXY", "ZYX"):
        sys.exit(f"Bone {name}: unsupported rotation order {order}.")
    rot = euler_to_matrix(list(props.get("Lcl Rotation", [0, 0, 0])), order)
    # FBX: local rotation = PreRotation * Rotation * PostRotation^-1 (pre/post are always XYZ).
    if "PreRotation" in props:
        rot = matmul(euler_to_matrix(list(props["PreRotation"])), rot)
    if "PostRotation" in props:
        rot = matmul(rot, transpose(euler_to_matrix(list(props["PostRotation"]))))
    for prop in ("RotationPivot", "ScalingPivot", "RotationOffset", "ScalingOffset"):
        if any(abs(v) > 1e-6 for v in props.get(prop, [0, 0, 0])):
            warnings.append(f"{name}: {prop} is not zero and is ignored")
    if any(abs(v - 1.0) > 1e-4 for v in props.get("Lcl Scaling", [1, 1, 1])):
        warnings.append(f"{name}: Lcl Scaling is not 1 and is ignored")
    return homogeneous(rot, list(props.get("Lcl Translation", [0, 0, 0])))


def bind_pose_relative(scene):
    """{bone name: parent->bone transform from the BindPose (global matrices)}, if the file has one."""
    names = {k: scene.name(o) for k, o in scene.models.items()}
    for o in scene.objects.values():
        if o.name == "Pose" and o.props[2] == "BindPose":
            glob = {}
            for pn in o.findall("PoseNode"):
                key = pn.find("Node").props[0]
                if key in scene.models:
                    m = pn.find("Matrix").props[0]
                    glob[key] = [[m[c * 4 + r] for c in range(4)] for r in range(4)]  # column-major
            rel = {}
            for key, g in glob.items():
                parent = scene.parent.get(key)
                if parent in glob:
                    pr = transpose([row[:3] for row in glob[parent][:3]])
                    rel[names[key]] = matmul(pr, [row[:3] for row in g[:3]])
            return rel
    return {}


def hand_nodes(scene, side, warnings):
    """MANUS raw-skeleton-like nodes (meters, hand frame) for one hand, from the FBX rest pose."""
    suffix = "_l" if side == "l" else "_r"
    by_name = {scene.name(o): k for k, o in scene.models.items() if o.props[2] == "LimbNode"}
    if f"hand{suffix}" not in by_name:
        return None
    to_m = scene.settings.get("UnitScaleFactor", [1.0])[0] / 100.0

    bones = [(f"hand{suffix}", "Hand", "Invalid", None)]
    for finger in FINGERS:
        segments = THUMB_SEGMENTS if finger == "thumb" else FINGER_SEGMENTS
        previous = f"hand{suffix}"
        for seg, joint_type in segments:
            bone = f"{finger}_{seg}{suffix}"
            if bone not in by_name:
                sys.exit(f"Bone {bone} not found in the FBX: export it from MANUS Core with the "
                         "'MANUS Polygon Avatar' skeleton.")
            if scene.name(scene.models[scene.parent[by_name[bone]]]) != previous:
                sys.exit(f"Bone {bone} is not a child of {previous}: unexpected skeleton structure.")
            bones.append((bone, f"Finger{finger.capitalize()}", joint_type, previous))
            previous = bone

    bind = bind_pose_relative(scene)
    ids = {name: i for i, (name, *_rest) in enumerate(bones)}
    # FBX bones flex about local Z (+Z on the left hand, -Z on the right hand); relabel the
    # bone axes to the SDK raw-skeleton convention used by build_hand_urdf (bone +X, flexion +Y).
    relabel_t = transpose(FBX_TO_SDK_AXES[side])
    world = {}
    nodes = []
    for name, chain, joint_type, parent in bones:
        local = local_rest_transform(scene, by_name[name], warnings)
        if parent is None:
            # The URDF root is the hand bone itself.
            world[name] = homogeneous([[1, 0, 0], [0, 1, 0], [0, 0, 1]], [0, 0, 0])
        else:
            world[name] = matmul(world[parent], local)
            if name in bind:
                diff = rotation_angle_deg([r[:3] for r in local[:3]], bind[name])
                if diff > 0.01:
                    warnings.append(f"{name}: static bone values differ from the BindPose by {diff:.2f} deg")
        w = world[name]
        nodes.append({
            "node_id": ids[name],
            "parent_id": ids[parent] if parent else ids[name],
            "chain_type": chain,
            "finger_joint_type": joint_type,
            "position_xyz": tuple(w[i][3] * to_m for i in range(3)),
            "rotation_wxyz": matrix_to_quat(matmul([r[:3] for r in w[:3]], relabel_t)),
        })
    return nodes


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("fbx", help="FBX file exported by MANUS Core")
    parser.add_argument("--side", choices=["left", "right", "both"], default="both")
    parser.add_argument("--out-dir", default=os.getcwd(), help="Output folder (default: current directory)")
    args = parser.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)

    scene = Scene(args.fbx)
    sides = ["l", "r"] if args.side == "both" else (["l"] if args.side == "left" else ["r"])
    failed = False
    for side in sides:
        side_name = "left" if side == "l" else "right"
        warnings = []
        nodes = hand_nodes(scene, side, warnings)
        if nodes is None:
            print(f"No {side_name} hand in {args.fbx}: URDF not written.")
            failed = True
            continue
        for w in warnings:
            print(f"Warning: {w}")
        print(f"--- {side_name} hand (FBX skeleton bind pose)")
        urdf_text = build_hand_urdf(side, nodes)
        filename = os.path.join(args.out_dir, f"{side_name}_hand_rest_pose.urdf")
        with open(filename, "w", encoding="utf-8") as f:
            f.write(urdf_text)
        print(f"Wrote {filename}")
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
