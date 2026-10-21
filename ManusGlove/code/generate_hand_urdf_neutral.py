"""
Same as `generate_hand_urdf.py --live`, but the raw-skeleton frame used as the URDF
zero pose is captured only when the MANUS ergonomics angles of that hand are all
within +/- --tolerance degrees of 0, continuously for --hold seconds.

By default only the flexion angles (*_stretch) are checked; --include-spread also
checks the *_spread angles, --exclude drops single angles (e.g. thumb_cmc_stretch).
Raw skeleton and ergonomics arrive on two separate streams: a raw frame is accepted
only if the latest ergonomics sample of the same glove is younger than --max-ergo-age.

This makes the captured reference pose repeatable; it does not prove that the URDF
axes match the internal MANUS model.

Outputs (in --out-dir):
  <side>_raw_skeleton_neutral.json  same format as generate_hand_urdf.py, usable with --from-json
  <side>_ergonomics_neutral.json    the 20 ergonomics angles (deg) of the accepted frame
  <side>_hand_neutral.urdf

Usage:
    python generate_hand_urdf_neutral.py --side right --manus-sdk-dir <MANUS_SDK>/Python
    python generate_hand_urdf_neutral.py --side left --tolerance 1.5 --hold 0.5
"""
import argparse
import json
import os
import sys
import threading
import time

from generate_hand_urdf import build_hand_urdf, connect_to_core, import_manus_sdk

# Order of the 20 per-hand values of ErgonomicsDataType (left = 0..19, right = 20..39).
ERGO_NAMES = [
    "thumb_cmc_spread", "thumb_cmc_stretch", "thumb_mcp_stretch", "thumb_ip_stretch",
    "index_mcp_spread", "index_mcp_stretch", "index_pip_stretch", "index_dip_stretch",
    "middle_mcp_spread", "middle_mcp_stretch", "middle_pip_stretch", "middle_dip_stretch",
    "ring_mcp_spread", "ring_mcp_stretch", "ring_pip_stretch", "ring_dip_stretch",
    "pinky_mcp_spread", "pinky_mcp_stretch", "pinky_pip_stretch", "pinky_dip_stretch",
]


def checked_indices(include_spread, exclude):
    unknown = set(exclude) - set(ERGO_NAMES)
    if unknown:
        sys.exit(f"Unknown ergonomics names in --exclude: {sorted(unknown)}. Valid: {ERGO_NAMES}")
    return [i for i, name in enumerate(ERGO_NAMES)
            if (include_spread or name.endswith("_stretch")) and name not in exclude]


def capture_neutral_nodes(side, indices, tolerance, hold, timeout, max_ergo_age,
                          connect_timeout=10.0, loopback_only=True, sdk_dir=None):
    """Return (nodes, ergonomics) for the first raw-skeleton frame of the given side
    whose checked ergonomics angles stayed within tolerance for `hold` seconds."""
    import_manus_sdk(sdk_dir)
    from manus_sdk import lib, ffi, ManusSDK
    from manus_sdk.generated._enums import (
        SDKReturnCode, AxisView, AxisPolarity, Side, HandMotion, ChainType, FingerJointType,
    )

    target_side = Side.Left if side == "l" else Side.Right
    offset = 0 if side == "l" else 20

    result = lib.CoreSdk_InitializeCore()
    if result != SDKReturnCode.Success:
        print(f"Failed to initialize SDK: {SDKReturnCode(result).name}")
        return None, None

    coord = ffi.new("CoordinateSystemVUH*")
    coord.view = AxisView.XFromViewer
    coord.up = AxisPolarity.PositiveZ
    coord.handedness = Side.Right
    coord.unitScale = 1.0
    result = lib.CoreSdk_InitializeCoordinateSystemWithVUH(coord[0], True)
    if result != SDKReturnCode.Success:
        print(f"Failed to initialize coordinate system: {SDKReturnCode(result).name}")
        lib.CoreSdk_ShutDown()
        return None, None

    lock = threading.Lock()
    latest_ergo = {}  # glove id -> (receive time, 20 angles in degrees)
    state = {"nodes": None, "infos": None, "ergo": None, "since": None, "worst": None}

    def on_ergonomics(stream):
        now = time.monotonic()
        with lock:
            for i in range(stream.dataCount):
                d = stream.data[i]
                if d.isUserID:
                    continue
                latest_ergo[d.id] = (now, [float(d.data[offset + k]) for k in range(20)])

    def on_raw_skeleton(stream_info):
        with lock:
            if state["nodes"] is not None:
                return
        for i in range(stream_info.skeletonsCount):
            info = ffi.new("RawSkeletonInfo*")
            if lib.CoreSdk_GetRawSkeletonInfo(i, info) != SDKReturnCode.Success:
                continue
            infos = ffi.new("NodeInfo[]", info.nodesCount)
            if lib.CoreSdk_GetRawSkeletonNodeInfoArray(info.gloveId, infos, info.nodesCount) != SDKReturnCode.Success:
                continue
            if infos[0].side != target_side:
                continue

            now = time.monotonic()
            with lock:
                sample = latest_ergo.get(info.gloveId)
                if sample is None or now - sample[0] > max_ergo_age:
                    state["since"], state["worst"] = None, None
                    return
                values = sample[1]
                worst = max(indices, key=lambda k: abs(values[k]))
                state["worst"] = (ERGO_NAMES[worst], values[worst])
                if abs(values[worst]) > tolerance:
                    state["since"] = None
                    return
                if state["since"] is None:
                    state["since"] = now
                if now - state["since"] < hold:
                    return

            nodes = ffi.new("SkeletonNode[]", info.nodesCount)
            if lib.CoreSdk_GetRawSkeletonData(i, nodes, info.nodesCount) != SDKReturnCode.Success:
                return
            with lock:
                state["nodes"] = [
                    (
                        n.id,
                        (n.transform.position.x, n.transform.position.y, n.transform.position.z),
                        (n.transform.rotation.w, n.transform.rotation.x, n.transform.rotation.y, n.transform.rotation.z),
                    )
                    for n in nodes
                ]
                state["infos"] = [(ni.nodeId, ni.parentId, ni.chainType, ni.fingerJointType) for ni in infos]
                state["ergo"] = list(values)
            return

    # Callbacks must be registered before connecting (Core subscribes at connection time).
    sdk = ManusSDK()
    sdk.register_ergonomics_callback(on_ergonomics)
    sdk.register_raw_skeleton_callback(on_raw_skeleton)

    print("Looking for a MANUS Core host...")
    if not connect_to_core(lib, ffi, SDKReturnCode, loopback_only, connect_timeout):
        print("Could not connect to MANUS Core. Is it running?")
        lib.CoreSdk_ShutDown()
        return None, None

    lib.CoreSdk_SetRawSkeletonHandMotion(HandMotion.Auto)

    side_name = "left" if side == "l" else "right"
    print(f"Connected. Bring the {side_name} hand to the MANUS zero pose "
          f"(checked angles within +/-{tolerance} deg for {hold} s)...")
    deadline = time.time() + timeout
    last_print = 0.0
    while time.time() < deadline:
        with lock:
            done = state["nodes"] is not None
            worst = state["worst"]
        if done:
            break
        if time.time() - last_print > 0.5:
            last_print = time.time()
            if worst is None:
                print("  waiting for synchronized raw-skeleton + ergonomics data...")
            else:
                print(f"  worst: {worst[0]} = {worst[1]:+.2f} deg")
        time.sleep(0.05)

    lib.CoreSdk_ShutDown()

    if state["nodes"] is None:
        print("The hand never stayed within tolerance before timeout.")
        return None, None

    positions = {nid: pos for nid, pos, rot in state["nodes"]}
    rotations = {nid: rot for nid, pos, rot in state["nodes"]}
    nodes = [
        {
            "node_id": nid,
            "parent_id": parent,
            "chain_type": ChainType(chain).name,
            "finger_joint_type": FingerJointType(joint).name,
            "position_xyz": positions[nid],
            "rotation_wxyz": rotations[nid],
        }
        for nid, parent, chain, joint in sorted(state["infos"])
    ]
    ergonomics = dict(zip(ERGO_NAMES, state["ergo"]))
    return nodes, ergonomics


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--side", choices=["left", "right", "both"], default="both")
    parser.add_argument("--tolerance", type=float, default=2.0, help="Max |angle| in degrees (default: 2.0)")
    parser.add_argument("--hold", type=float, default=0.5, help="Seconds the hand must stay within tolerance")
    parser.add_argument("--include-spread", action="store_true", help="Also check the *_spread angles")
    parser.add_argument("--exclude", nargs="*", default=[], help="Ergonomics names not to check")
    parser.add_argument("--max-ergo-age", type=float, default=0.1,
                        help="Max age (s) of the ergonomics sample paired with a raw-skeleton frame")
    parser.add_argument("--timeout", type=float, default=60.0, help="Seconds to wait for the zero pose")
    parser.add_argument("--connect-timeout", type=float, default=10.0, help="Seconds to wait for a Core connection")
    parser.add_argument("--remote", action="store_true", help="Search the network instead of localhost only")
    parser.add_argument("--manus-sdk-dir", help="Folder containing the 'manus_sdk' package. "
                        "Defaults to $MANUS_SDK_PYTHON_DIR")
    parser.add_argument("--out-dir", default=os.getcwd(), help="Output folder (default: current directory)")
    args = parser.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)

    indices = checked_indices(args.include_spread, args.exclude)
    if not indices:
        parser.error("no ergonomics angle left to check")
    print("Checked angles: " + ", ".join(ERGO_NAMES[i] for i in indices))

    sides = ["l", "r"] if args.side == "both" else (["l"] if args.side == "left" else ["r"])
    failed = False
    for side in sides:
        side_name = "left" if side == "l" else "right"
        nodes, ergonomics = capture_neutral_nodes(
            side, indices, args.tolerance, args.hold, args.timeout, args.max_ergo_age,
            args.connect_timeout, loopback_only=not args.remote, sdk_dir=args.manus_sdk_dir)
        if nodes is None:
            print(f"No zero-pose frame for the {side_name} hand: URDF not written.")
            failed = True
            continue

        for name, value in ergonomics.items():
            print(f"  {name:20s} {value:+7.2f} deg")

        with open(os.path.join(args.out_dir, f"{side_name}_raw_skeleton_neutral.json"), "w", encoding="utf-8") as f:
            json.dump(nodes, f, indent=2)
        with open(os.path.join(args.out_dir, f"{side_name}_ergonomics_neutral.json"), "w", encoding="utf-8") as f:
            json.dump({"tolerance_deg": args.tolerance, "hold_s": args.hold,
                       "checked": [ERGO_NAMES[i] for i in indices], "angles_deg": ergonomics}, f, indent=2)
        filename = os.path.join(args.out_dir, f"{side_name}_hand_neutral.urdf")
        with open(filename, "w", encoding="utf-8") as f:
            f.write(build_hand_urdf(side, nodes))
        print(f"Wrote {filename}")
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
