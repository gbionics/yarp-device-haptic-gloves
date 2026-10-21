"""
Generate the hand URDF from a MANUS Core CSV export (one file per hand), the same
way it is generated from the MANUS SDK.

The CSV export contains, for every frame, the MANUS raw skeleton (position and
rotation of the 25 hand nodes), i.e. the same data streamed by the SDK. This script
takes one frame, rebuilds the raw-skeleton nodes exactly as the SDK provides them to
generate_hand_urdf.py (meters, right-handed, node frames with the bone along +X) and
builds the URDF with the same function. It also writes them as <side>_raw_skeleton.json,
the same format saved by `generate_hand_urdf.py --live`, so
`generate_hand_urdf.py --from-json` gives the same URDF. Joint angles equal to zero
reproduce the pose of the chosen frame. The ergonomics angles are not used.

The CSV does not store the node hierarchy (the MANUS raw-skeleton one is used,
https://docs.manus-meta.com/3.2.0/Software/Skeletons/) nor some export settings:
  * units: --unit (default cm);
  * handedness of the coordinate system: set with --handedness or detected from the
    raw-skeleton palm, where MANUS places the thumb CMC on the palm side of the other
    CMCs (pose independent); left-handed data are converted to right-handed by
    flipping the X axis;
  * rotations: quaternions are used if present; for Euler angles the order and sign
    convention are detected by checking that every bone points along the same axis
    of its own node frame (as in the raw skeleton), otherwise the script stops.

Usage:
    python generate_hand_urdf_from_csv.py recording_R.csv
    python generate_hand_urdf_from_csv.py recording_L.csv --side left --frame 120 --out-dir <OUTPUT_DIR>
"""
import argparse
import csv
import itertools
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from generate_hand_urdf import build_hand_urdf, cross, normalize, sub  # noqa: E402

FINGERS = ("Thumb", "Index", "Middle", "Ring", "Pinky")
# CSV joint name -> MANUS FingerJointType
THUMB_NODES = (("CMC", "Metacarpal"), ("MCP", "Proximal"), ("DIP", "Distal"), ("TIP", "Tip"))
FINGER_NODES = (("CMC", "Metacarpal"), ("MCP", "Proximal"), ("PIP", "Intermediate"),
                ("DIP", "Distal"), ("TIP", "Tip"))
UNITS = {"m": 1.0, "cm": 0.01, "mm": 0.001}
MIN_BONE_AXIS_MATCH = math.cos(math.radians(3.0))
MIN_THUMB_OFFSET_M = 0.003


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def matvec(a, v):
    return [sum(a[i][k] * v[k] for k in range(3)) for i in range(3)]


def transpose(m):
    return [list(r) for r in zip(*m)]


def det(m):
    return dot(m[0], cross(m[1], m[2]))


def axis_rotation(axis, deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    if axis == "X":
        return [[1, 0, 0], [0, c, -s], [0, s, c]]
    if axis == "Y":
        return [[c, 0, s], [0, 1, 0], [-s, 0, c]]
    return [[c, -s, 0], [s, c, 0], [0, 0, 1]]


def euler_matrix(angles, order, sign):
    """Euler angles (X, Y, Z columns) applied in `order`, first axis first."""
    m = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    for axis in order:
        m = matmul(axis_rotation(axis, sign * angles["XYZ".index(axis)]), m)
    return m


def quat_to_matrix(q):
    n = math.sqrt(sum(c * c for c in q))
    w, x, y, z = (c / n for c in q)
    return [[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]]


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


def node_layout():
    """[(csv name, chain type, joint type, parent csv name)], in MANUS raw-skeleton id order."""
    layout = [("Hand", "Hand", "Invalid", None)]
    for finger in FINGERS:
        parent = "Hand"
        for csv_joint, joint_type in (THUMB_NODES if finger == "Thumb" else FINGER_NODES):
            name = f"{finger}_{csv_joint}"
            layout.append((name, f"Finger{finger}", joint_type, parent))
            parent = name
    return layout


def read_frame(path, frame):
    with open(path, encoding="utf-8-sig", newline="") as f:
        header = f.readline()
        f.seek(0)
        delimiter = csv.Sniffer().sniff(header, delimiters=",;\t").delimiter
        reader = csv.DictReader(f, delimiter=delimiter)
        if "Frame" not in (reader.fieldnames or []):
            sys.exit(f"{path}: no 'Frame' column, is it a MANUS Core CSV export?")
        for row in reader:
            if int(float(row["Frame"].replace(",", "."))) == frame:
                return row, reader.fieldnames
    sys.exit(f"{path}: frame {frame} not found.")


def num(row, column):
    try:
        return float(row[column].replace(",", "."))
    except (KeyError, AttributeError, ValueError):
        sys.exit(f"Column '{column}' is missing or not numeric: export the hand joint positions and "
                 "rotations for all fingers.")


def palm_axes(side, pos):
    """Unit vectors: radial (pinky -> index knuckle), distal (middle metacarpal), palmar if right-handed."""
    radial = normalize(sub(pos["Index_MCP"], pos["Pinky_MCP"]))
    distal = normalize(sub(pos["Middle_MCP"], pos["Middle_CMC"]))
    palmar = normalize(cross(radial, distal) if side == "r" else cross(distal, radial))
    return radial, distal, palmar


def thumb_palm_offset(side, pos):
    """Thumb CMC offset from the other CMCs along the palmar direction, assuming a right-handed frame.

    MANUS places the thumb CMC on the palm side of the other CMCs, so the offset is
    positive in a right-handed frame and negative in a left-handed (mirrored) one.
    """
    _, _, palmar = palm_axes(side, pos)
    center = [sum(pos[f"{f}_CMC"][i] for f in FINGERS[1:]) / 4 for i in range(3)]
    return dot(sub(pos["Thumb_CMC"], center), palmar)


def rotation_candidates(row, names, fields):
    if all(f"{n}_Rotation_W" in fields for n in names):
        quats = {n: tuple(num(row, f"{n}_Rotation_{a}") for a in "WXYZ") for n in names}
        yield "quaternion", {n: quat_to_matrix(q) for n, q in quats.items()}
        yield "quaternion (conjugate)", {n: transpose(quat_to_matrix(q)) for n, q in quats.items()}
        return
    eulers = {n: [num(row, f"{n}_Rotation_{a}") for a in "XYZ"] for n in names}
    for order in itertools.permutations("XYZ"):
        for sign in (1, -1):
            label = f"Euler {''.join(order)}" + ("" if sign > 0 else " (negated angles)")
            yield label, {n: euler_matrix(e, order, sign) for n, e in eulers.items()}


def bone_axis(rot, pos, layout):
    """Common bone direction in the node frames, and the worst match among all bones."""
    local = []
    for name, _, _, parent in layout:
        if parent in (None, "Hand"):
            continue
        d = normalize(sub(pos[name], pos[parent]))
        if d is not None:
            local.append(normalize(matvec(transpose(rot[parent]), d)))
    mean = normalize([sum(v[i] for v in local) for i in range(3)])
    if mean is None:
        return None, -1.0
    return mean, min(dot(v, mean) for v in local)


def sdk_local_axes(side, bone_axis_local, radial_hand_local):
    """Relabeling L of the node-frame axes to the SDK convention used by generate_hand_urdf.py.

    In the SDK raw skeleton (as captured by generate_hand_urdf.py) every bone points along
    +X of its node frame and, in the hand frame, the radial direction (pinky -> index) is
    -Y for the left hand and +Y for the right hand. L maps the CSV node axes to those
    (L * bone axis = +X); node rotations become R * L^T.
    """
    s = -1.0 if side == "l" else 1.0
    a = normalize(bone_axis_local)
    r = normalize(sub(radial_hand_local, [dot(radial_hand_local, a) * c for c in a]))
    source = [a, r, cross(a, r)]                                   # rows
    target = [[1.0, 0.0, 0.0], [0.0, s, 0.0], [0.0, 0.0, s]]       # rows: +X, s*Y, +X x s*Y
    exact = matmul(transpose(target), source)                      # L = T^T S (rows -> rows)
    rounded = [[float(round(v)) for v in row] for row in exact]
    if all(sum(abs(v) for v in row) == 1 for row in rounded) and abs(det(rounded) - 1) < 1e-9 and \
            max(abs(rounded[i][j] - exact[i][j]) for i in range(3) for j in range(3)) < 0.3:
        return rounded, True
    return exact, False


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("csv", help="CSV exported by MANUS Core (one hand)")
    parser.add_argument("--side", choices=["left", "right"], help="Hand in the file (default: from the _L/_R file suffix)")
    parser.add_argument("--frame", type=int, default=0, help="Value of the 'Frame' column to use (default: 0)")
    parser.add_argument("--unit", choices=sorted(UNITS), default="cm", help="Length unit of the export (default: cm)")
    parser.add_argument("--handedness", choices=["left", "right"],
                        help="Handedness of the export coordinate system (default: detected from the palm)")
    parser.add_argument("--out-dir", default=os.getcwd(), help="Output folder (default: current directory)")
    args = parser.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)

    stem = os.path.splitext(os.path.basename(args.csv))[0].upper()
    side_from_name = "left" if stem.endswith("_L") else "right" if stem.endswith("_R") else None
    side_name = args.side or side_from_name
    if side_name is None:
        sys.exit("Cannot tell the hand from the file name: use --side left or --side right.")
    if side_from_name and side_name != side_from_name:
        print(f"Warning: --side {side_name} but the file name suggests the {side_from_name} hand.")
    side = side_name[0]

    row, fields = read_frame(args.csv, args.frame)
    layout = node_layout()
    names = [n for n, *_ in layout]
    scale = UNITS[args.unit]
    pos = {n: [num(row, f"{n}_Position_{a}") * scale for a in "XYZ"] for n in names}

    offset = thumb_palm_offset(side, pos)
    detected = None
    if abs(offset) >= MIN_THUMB_OFFSET_M:
        detected = "right" if offset > 0 else "left"
        print(f"Coordinate system: {detected}-handed (thumb CMC {abs(offset) * 100:.2f} cm from the other CMCs)")
    handedness = args.handedness or detected
    if handedness is None:
        sys.exit("Cannot detect the coordinate-system handedness from the palm: use --handedness.")
    if args.handedness and detected and args.handedness != detected:
        print(f"Warning: --handedness {args.handedness} but the palm geometry suggests {detected}-handed.")

    # URDF is right-handed: flip X for left-handed exports (F p, F R F).
    f = (-1.0, 1.0, 1.0) if handedness == "left" else (1.0, 1.0, 1.0)
    pos = {n: [f[i] * p[i] for i in range(3)] for n, p in pos.items()}

    best = None
    for label, rots in rotation_candidates(row, names, fields):
        rots = {n: [[f[i] * f[j] * r[i][j] for j in range(3)] for i in range(3)] for n, r in rots.items()}
        axis, match = bone_axis(rots, pos, layout)
        if best is None or match > best[2]:
            best = (label, rots, match, axis)
    label, rots, match, axis = best
    if match < MIN_BONE_AXIS_MATCH:
        sys.exit(f"Cannot interpret the rotations (best: {label}, bones up to "
                 f"{math.degrees(math.acos(max(-1.0, match))):.1f} deg off a common axis). "
                 "Export the rotations as quaternions.")
    print(f"Rotations: {label} (all bones within {math.degrees(math.acos(min(1.0, match))):.2f} deg "
          f"of the node axis {tuple(round(c, 3) for c in axis)})")

    radial, _, _ = palm_axes(side, pos)
    relabel, exact = sdk_local_axes(side, axis, matvec(transpose(rots["Hand"]), radial))
    if not exact:
        print("Warning: the node axes do not map exactly onto the SDK convention; an approximate frame is used.")
    relabel_t = transpose(relabel)

    ids = {n: i for i, n in enumerate(names)}
    nodes = []
    for name, chain, joint_type, parent in layout:
        nodes.append({
            "node_id": ids[name],
            "parent_id": ids[parent] if parent else ids[name],
            "chain_type": chain,
            "finger_joint_type": joint_type,
            "position_xyz": tuple(pos[name]),
            "rotation_wxyz": matrix_to_quat(matmul(rots[name], relabel_t)),
        })

    middle = sum(math.dist(pos[a], pos[b]) for a, b in
                 (("Middle_MCP", "Middle_PIP"), ("Middle_PIP", "Middle_DIP"), ("Middle_DIP", "Middle_TIP")))
    if not 0.04 < middle < 0.15:
        print(f"Warning: middle finger length {middle:.3f} m is not plausible, check --unit.")

    json_path = os.path.join(args.out_dir, f"{side_name}_raw_skeleton.json")
    with open(json_path, "w", encoding="utf-8") as out:
        json.dump(nodes, out, indent=2)
    print(f"Wrote {json_path}")

    urdf_text = build_hand_urdf(side, nodes)
    urdf_path = os.path.join(args.out_dir, f"{side_name}_hand.urdf")
    with open(urdf_path, "w", encoding="utf-8") as out:
        out.write(urdf_text)
    print(f"Wrote {urdf_path}")


if __name__ == "__main__":
    main()
