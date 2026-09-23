#!/usr/bin/env python3
"""Hakoniwa point cloud reader asset.

Everything runs on the main thread: the Open3D window is created, then
hakopy.start() is called, and each cloud is drawn from inside the simulation
callback.

read_camera.py instead renders on the main thread and runs hakopy.start() on a
worker. That works for cv2, which needs only a brief slice per frame. Open3D
needs continuous time, and hakopy.start() holds the GIL between callbacks, so
the same split starved the render loop down to roughly one frame per second.
Rendering inside the callback keeps the drawing inside a slice Python already
owns. The cost is that a slow render slows simulation time; --render-every
trades displayed frames back for cadence.

This asset does NOT start Conductor; livox-mid360s-hakoniwa-asset.py owns it.

    python3 examples/sensors/livox_mid360s/read_point_cloud.py

Mouse drags orbit, the wheel zooms, Q closes the window.
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

import numpy as np
import open3d as o3d

from point_colors import by_range

import hakopy
from hakoniwa_pdu.impl.shm_communication_service import ShmCommunicationService
from hakoniwa_pdu.pdu_manager import PduManager
from hakoniwa_pdu.pdu_msgs.sensor_msgs.pdu_conv_PointCloud2 import pdu_to_py_PointCloud2

REPO_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_PDU_DEF = REPO_ROOT / "config/livox-mid360s-pdudef-compact.json"

ASSET = "Mid360SReader"

def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(
        description="Receive a Hakoniwa sensor_msgs/PointCloud2 PDU and show it with Open3D.")
    ap.add_argument("--config", default=str(DEFAULT_PDU_DEF), help="Hakoniwa pdudef.json")
    ap.add_argument("--robot", default="Mid360S")
    ap.add_argument("--pdu-name", default="point_cloud")
    ap.add_argument("--max-frames", type=int, default=0, help="0 runs until the window closes")
    ap.add_argument("--point-size", type=float, default=2.5)
    ap.add_argument("--render-every", type=int, default=1,
                    help="draw one frame in N; raise it if the display slows simulation time")
    ap.add_argument("--delta-usec", type=int, default=100_000,
                    help="Hakoniwa callback period; keep it at or below the "
                         "publisher's frame period or frames go unread")
    ap.add_argument("--headless", action="store_true",
                    help="consume and report clouds without opening a window")
    return ap.parse_args()


def finish(vis, stats) -> None:
    """Leave the simulation from inside a callback.

    hakopy offers a CONTROLLER asset no way to unregister: returning -1 from
    on_simulation_step does not make hakopy.start() return, and by the time the
    frame budget is spent the asset that owns Conductor may already have stopped
    it, leaving this process blocked. Report, release the window, and go.
    """
    print(f"\nINFO: received {stats['received']} clouds, drew {stats['drawn']}, "
          f"{stats['empty']} empty reads, {stats['invalid']} skipped before the "
          f"first write", flush=True)
    if vis is not None:
        vis.destroy_window()
    sys.stdout.flush()
    os._exit(0)


def main() -> int:
    args = parse_args()
    stats = {"received": 0, "empty": 0, "drawn": 0, "invalid": 0}

    vis = pcd = None
    if not args.headless:
        vis = o3d.visualization.Visualizer()
        if not vis.create_window(window_name="Livox Mid-360S (live from Hakoniwa PDU)",
                                 width=1400, height=900):
            print("ERROR: could not open a window; try --headless")
            return 1
        opt = vis.get_render_option()
        opt.point_size = args.point_size
        opt.background_color = np.array([0.06, 0.07, 0.09])
        vis.add_geometry(o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.5))
        pcd = o3d.geometry.PointCloud()

    pdu = PduManager()
    pdu.initialize(config_path=args.config, comm_service=ShmCommunicationService())
    pdu.start_service_nowait()

    def on_simulation_step(_context):
        # The PDU service must be pumped before a read; without this every read
        # returns empty even while the writer is publishing.
        pdu.run_nowait()
        raw = pdu.read_pdu_raw_data(args.robot, args.pdu_name)
        if raw is None or len(raw) == 0:
            stats["empty"] += 1
            return 0
        try:
            msg = pdu_to_py_PointCloud2(bytearray(raw))
        except Exception as exc:  # noqa: BLE001 - the channel is not written yet
            # A channel that has been created but never written reads back as
            # zeros, which is not an empty read and not a valid PDU. The C++
            # publisher reaches that state first, so skip until it writes.
            stats["invalid"] += 1
            if stats["invalid"] == 1:
                print(f"INFO: skipping invalid PDU until the publisher writes: {exc}",
                      flush=True)
            elif stats["invalid"] % 100 == 0:
                print(f"INFO: still waiting, {stats['invalid']} invalid reads", flush=True)
            return 0
        if msg.width == 0:
            stats["empty"] += 1
            return 0

        data = bytes(bytearray(msg.data))
        pts = np.frombuffer(data, dtype=np.float32).reshape(-1, 4)[:, :3].astype(np.float64)
        stats["received"] += 1

        if vis is not None and stats["received"] % args.render_every == 0:
            first = stats["drawn"] == 0
            pcd.points = o3d.utility.Vector3dVector(pts)
            pcd.colors = o3d.utility.Vector3dVector(by_range(np.linalg.norm(pts, axis=1)))
            if first:
                vis.add_geometry(pcd, reset_bounding_box=True)
            else:
                vis.update_geometry(pcd)
            stats["drawn"] += 1
            if not vis.poll_events():
                print("INFO: viewer window closed", flush=True)
                finish(vis, stats)
            vis.update_renderer()

        if stats["received"] % 10 == 0:
            print(f"frame {stats['received']:>4}  {len(pts):>6,} pts", flush=True)
        if args.max_frames and stats["received"] >= args.max_frames:
            finish(vis, stats)
        return 0

    callbacks = {
        "on_initialize": lambda c: 0,
        "on_simulation_step": on_simulation_step,
        "on_manual_timing_control": None,
        "on_reset": lambda c: 0,
    }
    # No conductor_start here: livox-mid360s-hakoniwa-asset.py owns Conductor.
    if not hakopy.asset_register(ASSET, args.config, callbacks, args.delta_usec,
                                 hakopy.HAKO_ASSET_MODEL_CONTROLLER):
        print("ERROR: asset_register failed")
        return 1

    print(f"INFO: reader registered on {args.robot}/{args.pdu_name}. WAITING for start",
          flush=True)
    try:
        hakopy.start()
    except KeyboardInterrupt:
        pass
    finish(vis, stats)
    return 0


if __name__ == "__main__":
    sys.exit(main())
