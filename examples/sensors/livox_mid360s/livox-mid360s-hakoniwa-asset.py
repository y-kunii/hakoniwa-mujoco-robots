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

Pass --viewer to watch the returns land on the scene in a MuJoCo viewer. That
is the only place the two can be seen together: read_point_cloud.py receives
the cloud without the model that produced it.

A composition that sizes the channel itself, such as a Hakoniwa Business Pack
Recipe, passes its own --config and --pdu-size instead.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

import numpy as np

import hakopy
import mujoco
import mujoco.viewer
from hakoniwa_pdu.impl.shm_communication_service import ShmCommunicationService
from hakoniwa_pdu.pdu_manager import PduManager
from hakoniwa_pdu.pdu_msgs.sensor_msgs.pdu_conv_PointCloud2 import py_to_pdu_PointCloud2
from hakoniwa_pdu.pdu_msgs.sensor_msgs.pdu_pytype_PointCloud2 import PointCloud2
from hakoniwa_pdu.pdu_msgs.sensor_msgs.pdu_pytype_PointField import PointField

from point_colors import by_range

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


class ViewerOverlay:
    """Draws the returns into the MuJoCo viewer, one sphere geom per point.

    The publisher is the only side that can do this: the reader receives the
    cloud alone and has no MuJoCo model to draw it against. Seeing the points
    on the scene is what makes an occlusion shadow readable.

    mjv_initGeom on every point costs about 44 ms at 24,000 points, enough to
    eat a 100 ms frame. Each geom is initialised once and only its position and
    colour are written afterwards, which measured roughly 40 percent cheaper.
    """

    def __init__(self, model, data, radius: float, decimate: int):
        self.decimate = max(1, decimate)
        self.radius = radius
        self.viewer = mujoco.viewer.launch_passive(model, data)
        self.capacity = 0
        self._size = np.array([radius, 0.0, 0.0])
        self._identity = np.eye(3).flatten()

    def _grow(self, n: int) -> None:
        scene = self.viewer.user_scn
        limit = min(n, scene.maxgeom)
        white = np.ones(4, dtype=np.float32)
        for i in range(self.capacity, limit):
            mujoco.mjv_initGeom(scene.geoms[i], mujoco.mjtGeom.mjGEOM_SPHERE,
                                self._size, np.zeros(3), self._identity, white)
        self.capacity = max(self.capacity, limit)

    def update(self, points: np.ndarray, distances: np.ndarray,
               origin: np.ndarray, rotation: np.ndarray) -> bool:
        """Draw one frame. False once the window has been closed."""
        if not self.viewer.is_running():
            return False
        points = points[::self.decimate]
        distances = distances[::self.decimate]
        world = points @ rotation.T + origin
        colors = by_range(distances)

        scene = self.viewer.user_scn
        n = min(len(world), scene.maxgeom)
        self._grow(n)
        with self.viewer.lock():
            for i in range(n):
                geom = scene.geoms[i]
                geom.pos[:] = world[i]
                geom.rgba[:3] = colors[i]
            scene.ngeom = n
        self.viewer.sync()
        return True

    def close(self) -> None:
        """Close the window and leave the process.

        Measured here: closing is instant and the viewer's thread is a daemon,
        yet the interpreter still dies during shutdown with the GLFW thread
        loaded. Skip that shutdown rather than end a clean run on a crash.
        """
        self.viewer.close()
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(0)


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

    overlay = state.get("overlay")
    if overlay is not None:
        origin, rotation = state["sensor"].origin(state["data"])
        if not overlay.update(scan.points, scan.distances, origin, rotation):
            print("INFO: viewer window closed; stopping", flush=True)
            return False

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
    ap.add_argument("--viewer", action="store_true",
                    help="show the returns on the scene in a MuJoCo viewer")
    ap.add_argument("--viewer-point-size", type=float, default=0.02,
                    help="drawn point radius in metres")
    ap.add_argument("--viewer-decimate", type=int, default=1,
                    help="draw every Nth point; raise it if drawing costs too much")
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
    print(f"capacity    : {budget:,} points at point_step {point_step}")
    if args.viewer:
        state["overlay"] = ViewerOverlay(model, data, args.viewer_point_size,
                                         args.viewer_decimate)
        note = f", every {args.viewer_decimate} points" if args.viewer_decimate > 1 else ""
        print(f"viewer      : on, radius {args.viewer_point_size} m{note}")
    print("", end="", flush=True)

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
    if state.get("overlay") is not None:
        state["overlay"].close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
