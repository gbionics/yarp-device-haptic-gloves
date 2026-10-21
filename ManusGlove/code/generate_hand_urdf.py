"""
Generate a URDF of the hand directly from the MANUS raw-skeleton node tree.

Every MANUS node becomes a URDF link with the same frame Core reports, and the
joint origins are the exact parent->child transforms of the captured pose, so
joint angles = 0 reproduces the captured pose.

MANUS raw-skeleton conventions (verified on captured data):
  * node position AND rotation are both expressed in one common frame
    (not parent-relative); this script converts them to parent-local.
  * every bone points along +X of its own node frame.
  * PIP, DIP, thumb MCP and thumb IP are exact hinges about +Y of the node frame
    (a positive rotation has the sign of the MANUS flex angle, on both hands).
  * node -> anatomical joint:
        fingers: Metacarpal = CMC (fixed), Proximal = MCP (spread+stretch),
                 Intermediate = PIP, Distal = DIP, Tip = fingertip (fixed)
        thumb:   Metacarpal = CMC (spread+stretch), Proximal = MCP,
                 Distal = IP, Tip = fingertip (fixed)
    which matches the 20 ergonomics values Core streams per hand.

Joint axes are expressed in the node frames (the joint frame at zero angle):
  * *_stretch: flexion about +Y of the node (MANUS "Flex"); PIP, DIP, thumb MCP and
    thumb IP are exact hinges about this axis in the MANUS raw skeleton.
  * *_mcp_spread: about the hand normal, applied before stretch, positive toward the
    thumb (-Z of the hand frame on the left hand, +Z on the right hand).
  * thumb_cmc_spread (palmar abduction): about the node Z axis, with the same sign rule;
    thumb_cmc_stretch (radial abduction): about -Y of the node, as MANUS uses the
    opposite alignment for the thumb CMC.
These axes follow the MANUS raw-skeleton motion and the sign of the MANUS
ergonomics angles (checked on recorded data). MANUS MCP and thumb CMC also include
a small twist that a 2-DOF joint cannot reproduce. The zero is the captured pose
(in MANUS ergonomics it is every bone aligned with the forward vector of the hand).

Modes (one is required, no generic/average hand is ever used):
  * --live: capture one frame from MANUS Core.
  * --from-json FILE: rebuild from a capture saved by --live.

--live needs the MANUS Python SDK (the `manus_sdk` package shipped in the
`Python` folder of the MANUS Core SDK, which requires `cffi`). It is looked up,
in order, in: --manus-sdk-dir, the MANUS_SDK_PYTHON_DIR environment variable,
the active Python environment.

Usage:
    python generate_hand_urdf.py --side right --live --manus-sdk-dir <MANUS_SDK>/Python
    python generate_hand_urdf.py --side right --from-json right_raw_skeleton.json
"""
import argparse
import json
import math
import os
import sys
import time
from xml.etree import ElementTree as ET
from xml.dom import minidom

CHAIN_TO_FINGER = {
    "FingerThumb": "thumb",
    "FingerIndex": "index",
    "FingerMiddle": "middle",
    "FingerRing": "ring",
    "FingerPinky": "pinky",
}

# (is_thumb, MANUS FingerJointType) -> (anatomical joint name, dofs at that node, in chain order)
JOINT_DOFS = {
    (True, "Metacarpal"): ("cmc", ("spread", "stretch")),
    (True, "Proximal"): ("mcp", ("stretch",)),
    (True, "Distal"): ("ip", ("stretch",)),
    (False, "Proximal"): ("mcp", ("spread", "stretch")),
    (False, "Intermediate"): ("pip", ("stretch",)),
    (False, "Distal"): ("dip", ("stretch",)),
}

LIMITS = {
    ("cmc", "spread"): (-0.5, 1.0),
    ("cmc", "stretch"): (-0.8, 0.8),
    ("mcp", "spread"): (-0.5, 0.5),
    ("mcp", "stretch"): (-0.5, 1.6),
    ("pip", "stretch"): (-0.2, 1.9),
    ("dip", "stretch"): (-0.2, 1.6),
    ("ip", "stretch"): (-0.3, 1.6),
}

LINK_RADIUS = 0.007


def fmt(values):
    return " ".join(f"{v:.6f}" for v in values)


def qmul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw)


def qinv(q):
    return (q[0], -q[1], -q[2], -q[3])


def qrot(q, v):
    return qmul(qmul(q, (0.0, *v)), qinv(q))[1:]


def sub(a, b):
    return tuple(x - y for x, y in zip(a, b))


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def normalize(v):
    n = math.sqrt(sum(x * x for x in v))
    return tuple(x / n for x in v) if n > 1e-9 else None


def quat_to_rpy(w, x, y, z):
    """Fixed-axis roll-pitch-yaw matching URDF's <origin rpy="..."/> convention."""
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return (roll, pitch, yaw)


def add_link(root, name, child_offset=None, radius=LINK_RADIUS):
    """Link with a sphere at its origin and, if given, a cylinder to its child's origin."""
    link = ET.SubElement(root, "link", name=name)
    shapes = [("sphere", (0, 0, 0), (0, 0, 0), {"radius": f"{radius * 1.2:.4f}"})]
    if child_offset is not None:
        length = math.sqrt(sum(c * c for c in child_offset))
        if length > 1e-6:
            vx, vy, vz = child_offset
            rpy = (0.0, math.atan2(math.hypot(vx, vy), vz), math.atan2(vy, vx))
            mid = tuple(c / 2 for c in child_offset)
            shapes.append(("cylinder", mid, rpy, {"radius": f"{radius:.4f}", "length": f"{length:.6f}"}))
    for tag in ("visual", "collision"):
        for shape, xyz, rpy, attrs in shapes:
            el = ET.SubElement(link, tag)
            ET.SubElement(el, "origin", xyz=fmt(xyz), rpy=fmt(rpy))
            geometry = ET.SubElement(el, "geometry")
            ET.SubElement(geometry, shape, **attrs)
    inertial = ET.SubElement(link, "inertial")
    ET.SubElement(inertial, "mass", value="0.01")
    ET.SubElement(inertial, "inertia", ixx="1e-6", ixy="0", ixz="0", iyy="1e-6", iyz="0", izz="1e-6")


def add_joint(root, name, joint_type, parent, child, xyz=(0, 0, 0), rpy=(0, 0, 0), axis=None, limit=None):
    joint = ET.SubElement(root, "joint", name=name, type=joint_type)
    ET.SubElement(joint, "parent", link=parent)
    ET.SubElement(joint, "child", link=child)
    ET.SubElement(joint, "origin", xyz=fmt(xyz), rpy=fmt(rpy))
    if axis is not None:
        ET.SubElement(joint, "axis", xyz=fmt(axis))
    if limit is not None:
        ET.SubElement(joint, "limit", lower=f"{limit[0]:.4f}", upper=f"{limit[1]:.4f}",
                      effort="1.0", velocity="2.0")


def build_hand_urdf(side, nodes, verbose=True):
    """Build the URDF from MANUS-style nodes (positions/rotations in one common frame)."""
    prefix = side
    by_id = {n["node_id"]: n for n in nodes}
    root_node = next(n for n in nodes if n["chain_type"] == "Hand")

    # Re-express everything in the Hand node's frame, so the URDF root is the wrist.
    # SDK quaternions are float32 and slightly off unit norm: normalize them first.
    quat = {nid: normalize(tuple(n["rotation_wxyz"])) for nid, n in by_id.items()}
    qh_inv = qinv(quat[root_node["node_id"]])
    ph = tuple(root_node["position_xyz"])
    pos = {nid: qrot(qh_inv, sub(n["position_xyz"], ph)) for nid, n in by_id.items()}
    rot = {nid: qmul(qh_inv, quat[nid]) for nid in by_id}

    children = {}
    for n in nodes:
        if n["node_id"] != n["parent_id"]:
            children.setdefault(n["parent_id"], []).append(n["node_id"])

    # Joint axes in the node frame (= joint frame at zero angle), fitted on MANUS raw-skeleton motion:
    # flexion is about +Y of the node (exact hinge for PIP/DIP/thumb MCP/IP), MCP spread is about the
    # hand normal and thumb CMC uses the node axes with the opposite stretch sign (as in MANUS).
    # Spread signs move the finger toward the thumb: -Z on the left hand, +Z on the right hand.
    spread_sign = -1.0 if side == "l" else 1.0

    def joint_axes(nid, finger, joint_name):
        if finger == "thumb" and joint_name == "cmc":
            return {"spread": (0.0, 0.0, spread_sign), "stretch": (0.0, -1.0, 0.0)}
        return {"spread": qrot(qinv(rot[nid]), (0.0, 0.0, spread_sign)), "stretch": (0.0, 1.0, 0.0)}

    def link_name(n):
        if n["chain_type"] == "Hand":
            return f"{prefix}_hand"
        return f"{prefix}_{CHAIN_TO_FINGER[n['chain_type']]}_{n['finger_joint_type'].lower()}"

    robot = ET.Element("robot", name=f"{prefix}_hand")
    add_link(robot, f"{prefix}_hand", radius=0.012)

    if verbose:
        print(f"{'Link':<26}{'Parent':<26}{'xyz in parent (m)':<36}rpy in parent (rad)")
    for n in sorted(nodes, key=lambda n: n["node_id"]):
        nid, pid = n["node_id"], n["parent_id"]
        finger = CHAIN_TO_FINGER.get(n["chain_type"])
        if finger is None or nid == pid:
            continue
        name, parent_name = link_name(n), link_name(by_id[pid])

        q_parent_inv = qinv(rot[pid])
        xyz = qrot(q_parent_inv, sub(pos[nid], pos[pid]))
        rpy = quat_to_rpy(*qmul(q_parent_inv, rot[nid]))
        kids = [c for c in children.get(nid, []) if CHAIN_TO_FINGER.get(by_id[c]["chain_type"]) == finger]
        child_offset = qrot(qinv(rot[nid]), sub(pos[kids[0]], pos[nid])) if kids else None
        if verbose:
            print(f"{name:<26}{parent_name:<26}{fmt(xyz):<36}{fmt(rpy)}")

        dofs_spec = JOINT_DOFS.get((finger == "thumb", n["finger_joint_type"]))
        if dofs_spec is None:
            add_link(robot, name, child_offset)
            add_joint(robot, f"{name}_fixed", "fixed", parent_name, name, xyz, rpy)
            continue

        joint_name, dofs = dofs_spec
        axes = joint_axes(nid, finger, joint_name)

        current_parent = parent_name
        for i, dof in enumerate(dofs):
            last = i == len(dofs) - 1
            child = name if last else f"{prefix}_{finger}_{joint_name}_{dof}"
            add_link(robot, child, child_offset if last else None)
            origin_xyz, origin_rpy = (xyz, rpy) if i == 0 else ((0, 0, 0), (0, 0, 0))
            add_joint(robot, f"{prefix}_{finger}_{joint_name}_{dof}", "revolute", current_parent, child,
                      origin_xyz, origin_rpy, axis=axes[dof], limit=LIMITS[(joint_name, dof)])
            current_parent = child

    xml_str = ET.tostring(robot, encoding="unicode")
    return minidom.parseString(xml_str).toprettyxml(indent="  ")


# ---------------------------------------------------------------------------
# Live capture: read real bone lengths from a connected MANUS Core / glove.
# ---------------------------------------------------------------------------
def connect_to_core(lib, ffi, SDKReturnCode, loopback_only, connect_timeout):
    """Look for a Core host and connect to it (mirrors minimal_client.connect())."""
    deadline = time.time() + connect_timeout
    while time.time() < deadline:
        if lib.CoreSdk_LookForHosts(1, loopback_only) != SDKReturnCode.Success:
            time.sleep(0.5)
            continue

        count = ffi.new("uint32_t*")
        if lib.CoreSdk_GetNumberOfAvailableHostsFound(count) != SDKReturnCode.Success or count[0] == 0:
            time.sleep(0.5)
            continue

        hosts = ffi.new("ManusHost[]", count[0])
        if lib.CoreSdk_GetAvailableHostsFound(hosts, count[0]) != SDKReturnCode.Success:
            time.sleep(0.5)
            continue

        if lib.CoreSdk_ConnectToHost(hosts[0]) == SDKReturnCode.Success:
            return True
        time.sleep(0.5)
    return False


def import_manus_sdk(sdk_dir=None):
    """Make the MANUS Python SDK importable. `sdk_dir` is the folder containing
    the `manus_sdk` package (i.e. the `Python` folder of the MANUS Core SDK)."""
    sdk_dir = sdk_dir or os.environ.get("MANUS_SDK_PYTHON_DIR")
    if sdk_dir:
        sdk_dir = os.path.abspath(os.path.expanduser(sdk_dir))
        if not os.path.isdir(os.path.join(sdk_dir, "manus_sdk")):
            sys.exit(f"'manus_sdk' package not found in {sdk_dir}. "
                     "Point --manus-sdk-dir / MANUS_SDK_PYTHON_DIR to the 'Python' folder of the MANUS Core SDK.")
        sys.path.insert(0, sdk_dir)
    try:
        import manus_sdk
    except ImportError as e:
        sys.exit(f"Cannot import the MANUS Python SDK ({e}).\n"
                 "Pass --manus-sdk-dir <MANUS_SDK>/Python or set MANUS_SDK_PYTHON_DIR.")
    if manus_sdk.lib is None:
        sys.exit("The MANUS Python SDK was found but its native bindings could not be loaded "
                 f"for Python {sys.version_info.major}.{sys.version_info.minor} (see the warning above).")
    return manus_sdk


def capture_live_nodes(side, timeout, connect_timeout=10.0, loopback_only=True, sdk_dir=None):
    """Connect to Core and return one raw-skeleton frame for the given glove side
    as a list of node dicts (exactly what Core sent), or None on timeout."""
    import_manus_sdk(sdk_dir)
    from manus_sdk import lib, ffi, ManusSDK
    from manus_sdk.generated._enums import (
        SDKReturnCode, AxisView, AxisPolarity, Side, HandMotion, ChainType, FingerJointType,
    )

    target_side = Side.Left if side == "l" else Side.Right

    # Raw skeleton data requires an actual Core connection (Local/Remote),
    # not CoreSdk_InitializeIntegrated() which runs standalone without gloves.
    result = lib.CoreSdk_InitializeCore()
    if result != SDKReturnCode.Success:
        print(f"Failed to initialize SDK: {SDKReturnCode(result).name}")
        return None

    # Coordinate system must be set up before connecting, or Core rejects the connection.
    coord = ffi.new("CoordinateSystemVUH*")
    coord.view = AxisView.XFromViewer
    coord.up = AxisPolarity.PositiveZ
    coord.handedness = Side.Right
    coord.unitScale = 1.0
    result = lib.CoreSdk_InitializeCoordinateSystemWithVUH(coord[0], True)
    if result != SDKReturnCode.Success:
        print(f"Failed to initialize coordinate system: {SDKReturnCode(result).name}")
        lib.CoreSdk_ShutDown()
        return None

    captured = {"nodes": None, "infos": None}

    def on_raw_skeleton(stream_info):
        if captured["nodes"] is not None:
            return
        for i in range(stream_info.skeletonsCount):
            info = ffi.new("RawSkeletonInfo*")
            if lib.CoreSdk_GetRawSkeletonInfo(i, info) != SDKReturnCode.Success:
                continue
            nodes = ffi.new("SkeletonNode[]", info.nodesCount)
            if lib.CoreSdk_GetRawSkeletonData(i, nodes, info.nodesCount) != SDKReturnCode.Success:
                continue
            infos = ffi.new("NodeInfo[]", info.nodesCount)
            if lib.CoreSdk_GetRawSkeletonNodeInfoArray(info.gloveId, infos, info.nodesCount) != SDKReturnCode.Success:
                continue
            if infos[0].side != target_side:
                continue
            captured["nodes"] = [
                (
                    n.id,
                    (n.transform.position.x, n.transform.position.y, n.transform.position.z),
                    (n.transform.rotation.w, n.transform.rotation.x, n.transform.rotation.y, n.transform.rotation.z),
                )
                for n in nodes
            ]
            captured["infos"] = [(ni.nodeId, ni.parentId, ni.chainType, ni.fingerJointType) for ni in infos]
            return

    # The callback must be registered before connecting: Core only subscribes a
    # client to the streams that already have a callback at connection time.
    sdk = ManusSDK()
    sdk.register_raw_skeleton_callback(on_raw_skeleton)

    print("Looking for a MANUS Core host...")
    if not connect_to_core(lib, ffi, SDKReturnCode, loopback_only, connect_timeout):
        print("Could not connect to MANUS Core. Is it running?")
        lib.CoreSdk_ShutDown()
        return None

    lib.CoreSdk_SetRawSkeletonHandMotion(HandMotion.Auto)

    print(f"Connected. Hold the {('left' if side == 'l' else 'right')} hand flat and still...")
    deadline = time.time() + timeout
    while captured["nodes"] is None and time.time() < deadline:
        time.sleep(0.05)

    lib.CoreSdk_ShutDown()

    if captured["nodes"] is None:
        print("No raw-skeleton data received before timeout.")
        return None

    positions = {nid: pos for nid, pos, rot in captured["nodes"]}
    rotations = {nid: rot for nid, pos, rot in captured["nodes"]}
    return [
        {
            "node_id": nid,
            "parent_id": parent,
            "chain_type": ChainType(chain).name,
            "finger_joint_type": FingerJointType(joint).name,
            "position_xyz": positions[nid],
            "rotation_wxyz": rotations[nid],
        }
        for nid, parent, chain, joint in sorted(captured["infos"])
    ]


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--side", choices=["left", "right", "both"], default="both")
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--live", action="store_true", help="Capture the hand from MANUS Core")
    source.add_argument("--from-json", help="Rebuild from a capture saved by --live (single side)")
    parser.add_argument("--timeout", type=float, default=10.0, help="Seconds to wait for live skeleton data")
    parser.add_argument("--connect-timeout", type=float, default=10.0, help="Seconds to wait for a Core connection")
    parser.add_argument("--remote", action="store_true", help="Search the network instead of localhost only")
    parser.add_argument("--manus-sdk-dir", help="Folder containing the 'manus_sdk' package "
                        "(the 'Python' folder of the MANUS Core SDK). Defaults to $MANUS_SDK_PYTHON_DIR")
    parser.add_argument("--out-dir", default=os.getcwd(), help="Output folder (default: current directory)")
    args = parser.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)

    sides = ["l", "r"] if args.side == "both" else (["l"] if args.side == "left" else ["r"])
    if args.from_json and len(sides) != 1:
        parser.error("--from-json needs --side left or --side right")

    failed = False
    for side in sides:
        side_name = "left" if side == "l" else "right"
        if args.from_json:
            with open(args.from_json, encoding="utf-8") as f:
                nodes = json.load(f)
        else:
            nodes = capture_live_nodes(side, args.timeout, args.connect_timeout, loopback_only=not args.remote,
                                       sdk_dir=args.manus_sdk_dir)
            if nodes is None:
                print(f"No MANUS data for the {side_name} hand: URDF not written.")
                failed = True
                continue
            dump_path = os.path.join(args.out_dir, f"{side_name}_raw_skeleton.json")
            with open(dump_path, "w", encoding="utf-8") as f:
                json.dump(nodes, f, indent=2)
            print(f"Wrote raw MANUS capture to {dump_path}")
        urdf_text = build_hand_urdf(side, nodes)
        filename = os.path.join(args.out_dir, f"{side_name}_hand.urdf")
        with open(filename, "w", encoding="utf-8") as f:
            f.write(urdf_text)
        print(f"Wrote {filename}")
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
