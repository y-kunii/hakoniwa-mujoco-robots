#!/usr/bin/env python3
"""Hakoniwa asset that publishes the Livox Mid-360S point cloud as a PDU.

Companion to ``livox_mid360s_sensor.py``, in the same shape as
``examples/sensors/color_camera/color-camera-hakoniwa-asset.cpp``: the sensor
module owns ray casting, the range gate and the accuracy model, and this asset
owns what belongs to Hakoniwa - registration, Conductor ownership, simulation
timing, and packing a scan into a ``sensor_msgs/PointCloud2`` PDU.

It owns Conductor. Read the cloud back with ``read_point_cloud.py``, which must
not start Conductor.

Every path defaults to what this repository ships, so the example runs from the
repository root with no arguments:

    python3 examples/sensors/livox_mid360s/livox-mid360s-hakoniwa-asset.py

A composition that sizes the channel itself, such as a Hakoniwa Business Pack
Recipe, passes its own --config and --pdu-size instead.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

import hakopy
import mujoco
from hakoniwa_pdu.impl.shm_communication_service import ShmCommunicationService
from hakoniwa_pdu.pdu_manager import PduManager
from hakoniwa_pdu.pdu_msgs.sensor_msgs.pdu_conv_PointCloud2 import py_to_pdu_PointCloud2
from hakoniwa_pdu.pdu_msgs.sensor_msgs.pdu_pytype_PointCloud2 import PointCloud2
from hakoniwa_pdu.pdu_msgs.sensor_msgs.pdu_pytype_PointField import PointField

REPO_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_PDU_DEF = REPO_ROOT / "config/livox-mid360s-pdudef-compact.json"
DEFAULT_PROFILE = REPO_ROOT / "config/sensors/lidar/livox-mid360s.json"
DEFAULT_SCENE = REPO_ROOT / "models/sensors/lidar_3d/livox-mid360s-sample.xml"

ROBOT = "Mid360S"
CHANNEL = "point_cloud"
ASSET = "Mid360SSensor"
FLOAT32 = 7
# Measured: the serialised PointCloud2 envelope is this many bytes regardless of
# point count, so channel size is ENVELOPE_BYTES + points * point_step.
ENVELOPE_BYTES = 760

state: dict = {}


def build_cloud(points: np.ndarray, stamp_ns: int, frame_id: str, step: int) -> PointCloud2:
    n = len(points)
    msg = PointCloud2()
    msg.header.frame_id = frame_id
    msg.header.stamp.sec = stamp_ns // 1_000_000_000
    msg.header.stamp.nanosec = stamp_ns % 1_000_000_000
    msg.height = 1
    msg.width = n
    msg.is_bigendian = False
    msg.is_dense = True
    msg.point_step = step
    msg.row_step = step * n
    for i, name in enumerate(("x", "y", "z", "intensity")):
        f = PointField()
        f.name, f.offset, f.datatype, f.count = name, 4 * i, FLOAT32, 1
        msg.fields.append(f)
    xyzi = np.zeros((n, 4), dtype=np.float32)
    xyzi[:, :3] = points
    xyzi[:, 3] = 1.0
    msg.data = list(xyzi.tobytes())
    return msg


def publish_frame() -> bool:
    scan = state["sensor"].scan(state["data"])

    pts = scan.points
    dropped = 0
    if len(pts) > state["max_points"]:
        dropped = len(pts) - state["max_points"]
        pts = pts[: state["max_points"]]

    msg = build_cloud(pts, hakopy.simulation_time() * 1000,
                      state["sensor"].frame_id, state["point_step"])
    raw = py_to_pdu_PointCloud2(msg)
    if len(raw) > state["pdu_size"]:
        print(f"ERROR: serialised {len(raw)} bytes exceeds pdu_size {state['pdu_size']}")
        return False
    if not state["pdu"].flush_pdu_raw_data_nowait(ROBOT, CHANNEL, raw):
        print("ERROR: flush_pdu_raw_data_nowait failed")
        return False

    state["frames"] += 1
    state["last_bytes"] = len(raw)
    if state["frames"] <= 3 or state["frames"] % 10 == 0:
        note = f"  (dropped {dropped:,} over budget)" if dropped else ""
        print(f"{hakopy.simulation_time():>12} us  frame {state['frames']:>4}  "
              f"{len(pts):>6,} pts  {len(raw):>7,} B{note}", flush=True)
    return True


def on_initialize(context):  # noqa: ARG001
    print(f"INFO: asset initialized, publishing {ROBOT}/{CHANNEL}", flush=True)
    return 0


def on_reset(context):  # noqa: ARG001
    state["frames"] = 0
    return 0


def on_manual_timing_control(context):  # noqa: ARG001
    print("INFO: simulation running; publishing point clouds", flush=True)
    step_usec = int(1_000_000 / state["frame_rate"])
    while True:
        if not publish_frame():
            break
        if not hakopy.usleep(step_usec):
            break
        if state["max_frames"] and state["frames"] >= state["max_frames"]:
            print(f"INFO: reached --max-frames {state['max_frames']}", flush=True)
            break
    print(f"INFO: published {state['frames']} frames, last {state['last_bytes']:,} bytes",
          flush=True)
    return 0


def declared_channel_size(pdu_def: Path, robot: str, channel: str) -> int:
    """The pdu_size a pdudef declares for one robot channel.

    Resolves pdudef -> pdutypes the way the Hakoniwa runtime does, so the asset
    budgets against exactly the channel the runtime will open.
    """
    definition = json.loads(pdu_def.read_text(encoding="utf-8"))
    try:
        types_id = next(r["pdutypes_id"] for r in definition["robots"] if r["name"] == robot)
        types_rel = next(p["path"] for p in definition["paths"] if p["id"] == types_id)
    except StopIteration:
        raise SystemExit(f"{pdu_def}: no channel set for robot {robot!r}") from None
    entries = json.loads((pdu_def.parent / types_rel).read_text(encoding="utf-8"))
    for entry in entries:
        if entry["name"] == channel:
            return int(entry["pdu_size"])
    raise SystemExit(f"{pdu_def.parent / types_rel}: no channel named {channel!r}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", default=str(DEFAULT_PDU_DEF), help="Hakoniwa pdudef.json")
    ap.add_argument("--sensor-root", default=str(REPO_ROOT),
                    help="repository root providing python/livox_mid360s_sensor.py")
    ap.add_argument("--profile", default=str(DEFAULT_PROFILE), help="lidar_3d profile JSON")
    ap.add_argument("--scene", default=str(DEFAULT_SCENE), help="MJCF scene")
    ap.add_argument("--pdu-size", type=int, default=None,
                    help="channel bytes; read from the pdudef's pdutypes when omitted")
    ap.add_argument("--max-frames", type=int, default=0)
    ap.add_argument("--no-noise", action="store_true")
    args = ap.parse_args()

    sys.path.insert(0, str(Path(args.sensor_root).resolve() / "python"))
    from livox_mid360s_sensor import LivoxMid360SSensor  # noqa: E402

    model = mujoco.MjModel.from_xml_path(args.scene)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    sensor = LivoxMid360SSensor(model, args.profile, apply_noise=not args.no_noise)

    profile = json.loads(Path(args.profile).read_text(encoding="utf-8"))
    point_step = int(profile["pdu_config"].get("point_step", 16))
    pdu_size = args.pdu_size
    if pdu_size is None:
        # Read the channel the pdudef already declares rather than restating it
        # here: a second copy of the size is what drifts.
        pdu_size = declared_channel_size(Path(args.config), ROBOT, CHANNEL)
    budget = (pdu_size - ENVELOPE_BYTES) // point_step

    state.update(
        data=data, sensor=sensor, pdu_size=pdu_size, point_step=point_step,
        max_points=budget, max_frames=args.max_frames,
        frame_rate=sensor.pattern.frame_rate, frames=0, last_bytes=0,
    )

    print(f"sensor root : {args.sensor_root}")
    print(f"profile     : {sensor.name}  pattern={sensor.pattern.kind}")
    print(f"scene       : {args.scene}")
    print(f"channel     : {ROBOT}/{CHANNEL}  pdu_size={pdu_size:,}")
    print(f"capacity    : {budget:,} points at point_step {point_step}", flush=True)

    pdu = PduManager()
    pdu.initialize(config_path=args.config, comm_service=ShmCommunicationService())
    pdu.start_service_nowait()
    state["pdu"] = pdu

    delta_usec = int(1_000_000 / sensor.pattern.frame_rate)
    hakopy.conductor_start(delta_usec, delta_usec)  # this asset owns Conductor
    callbacks = {
        "on_initialize": on_initialize,
        "on_simulation_step": None,
        "on_manual_timing_control": on_manual_timing_control,
        "on_reset": on_reset,
    }
    if not hakopy.asset_register(ASSET, args.config, callbacks, delta_usec,
                                 hakopy.HAKO_ASSET_MODEL_PLANT):
        print("ERROR: asset_register failed")
        hakopy.conductor_stop()
        return 1

    print("INFO: registered. WAITING for hako-cmd start", flush=True)
    hakopy.start()
    hakopy.conductor_stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
