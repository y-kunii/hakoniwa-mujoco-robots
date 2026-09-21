#!/usr/bin/env python3
"""Build and inspect ray-angle tables for the lidar_3d 'table' scan pattern.

A table is an (N, 2) float32 array of (azimuth, elevation) in radians, stored in
acquisition order. ``livox_mid360s_sensor.py`` replays it, wrapping when it runs
out, which is what reproduces non-repetitive scanning.

Two sources are supported.

``from-cloud``
    Convert a recorded point cloud from the physical sensor. Each point's
    direction is the ray that produced it, so a recording IS a scan-pattern
    measurement. This is the only source that can honestly be called measured
    hardware behaviour. Input is CSV or NPY with columns x, y, z in the sensor
    frame, optionally followed by a timestamp used to restore acquisition order.

``from-mujoco-lidar``
    Export the table bundled with the third-party ``mujoco-lidar`` package. Its
    origin is not documented anywhere in that package, so anything derived from
    it is a hypothesis, not a measurement. Recorded as such in the report.

    python3 python/livox_scan_pattern_tool.py from-cloud recording.csv \\
        --out scan_patterns/livox-mid360s.npy
    python3 python/livox_scan_pattern_tool.py inspect scan_patterns/*.npy
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np


def _load_points(path: Path) -> np.ndarray:
    if path.suffix.lower() == ".npy":
        arr = np.load(path)
    else:
        arr = np.loadtxt(path, delimiter=",", dtype=np.float64, ndmin=2)
    arr = np.asarray(arr, dtype=np.float64)
    if arr.ndim != 2 or arr.shape[1] < 3:
        raise SystemExit(f"{path}: expected at least 3 columns (x, y, z), got {arr.shape}")
    return arr


def from_cloud(args) -> int:
    arr = _load_points(Path(args.input))
    xyz = arr[:, :3]

    if args.time_column is not None:
        if arr.shape[1] <= args.time_column:
            raise SystemExit(f"--time-column {args.time_column} is beyond the {arr.shape[1]} columns")
        order = np.argsort(arr[:, args.time_column], kind="stable")
        xyz = xyz[order]
        ordering = f"sorted by column {args.time_column}"
    else:
        ordering = "file order (assumed acquisition order)"

    r = np.linalg.norm(xyz, axis=1)
    keep = r > args.min_range
    dropped = int((~keep).sum())
    xyz, r = xyz[keep], r[keep]

    az = np.arctan2(xyz[:, 1], xyz[:, 0])
    el = np.arcsin(np.clip(xyz[:, 2] / r, -1.0, 1.0))
    table = np.stack([az, el], axis=1).astype(np.float32)

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    np.save(out, table)

    print(f"input        : {args.input}")
    print(f"ordering     : {ordering}")
    print(f"points       : {len(arr):,} read, {dropped:,} dropped below {args.min_range} m")
    print(f"table        : {len(table):,} rays -> {out}")
    _report(table, len(table))
    print()
    print("Record the provenance in the profile's ScanPattern.TableProvenance, for example:")
    print(f'  "Measured from {Path(args.input).name} on <date>, <sensor serial>, '
          f'<motor mode>, static mount."')
    return 0


def from_mujoco_lidar(args) -> int:
    try:
        import mujoco_lidar
    except ImportError:
        raise SystemExit(
            "mujoco-lidar is not installed. It is an optional source for this tool only; "
            "the sensor itself does not depend on it."
        )
    src = Path(mujoco_lidar.__file__).parent / "scan_mode" / f"{args.model}.npy"
    if not src.exists():
        raise SystemExit(f"no bundled table for {args.model!r}: {src}")

    table = np.asarray(np.load(src), dtype=np.float32)
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    np.save(out, table)

    print(f"source       : {src}")
    print(f"table        : {len(table):,} rays -> {out}")
    _report(table, len(table))
    print()
    print("PROVENANCE WARNING")
    print("  mujoco-lidar ships this table with no statement of how it was produced.")
    print("  A hardware recording, an SDK export and a synthesised model are all")
    print("  consistent with what is published. Do not present results from it as")
    print("  measured sensor behaviour. Suggested ScanPattern.TableProvenance:")
    print(f'    "Exported from mujoco-lidar scan_mode/{args.model}.npy. Origin not')
    print('     documented upstream. UNVERIFIED against hardware."')
    return 0


def _report(table: np.ndarray, samples_hint: int) -> None:
    az = np.degrees(np.mod(table[:, 0], 2 * np.pi))
    el = np.degrees(table[:, 1])
    print(f"azimuth      : {az.min():7.2f} .. {az.max():7.2f} deg")
    print(f"elevation    : {el.min():7.2f} .. {el.max():7.2f} deg  "
          f"(span {el.max() - el.min():.2f})")

    # Work in radians throughout, then convert once: mixing units here silently
    # inflates the step and makes the ordering test meaningless.
    az_rad, el_rad = np.unwrap(table[:, 0].astype(np.float64)), table[:, 1].astype(np.float64)
    step = np.degrees(np.hypot(np.abs(np.diff(az_rad)), np.abs(np.diff(el_rad))))
    if len(step):
        rng = np.random.default_rng(0)
        perm = rng.permutation(len(table))
        shuffled = np.degrees(np.hypot(
            np.abs(np.diff(np.unwrap(table[perm, 0].astype(np.float64)))),
            np.abs(np.diff(table[perm, 1].astype(np.float64)))))
        ordered = np.median(step) < np.median(shuffled) / 5
        print(f"consecutive  : median {np.median(step):.3f} deg between entries, "
              f"{np.median(shuffled):.1f} deg if shuffled -> "
              f"{'acquisition order' if ordered else 'ordering unclear'}")

    sectors = 180
    edges = np.linspace(0, 360, sectors + 1)
    idx = np.clip(np.digitize(az, edges) - 1, 0, sectors - 1)
    lowest = np.full(sectors, np.inf)
    np.minimum.at(lowest, idx, el)
    finite = lowest[np.isfinite(lowest)]
    if len(finite):
        spread = finite.max() - finite.min()
        print(f"lowest elev  : {finite.min():.2f} .. {finite.max():.2f} deg across azimuth "
              f"(spread {spread:.2f})")
        if spread > 0.5:
            print("               -> the floor blind zone is a lobe, not a circle. "
                  "Check this against hardware before planning with it.")


def _inspect(args) -> int:
    for path in args.tables:
        p = Path(path)
        table = np.asarray(np.load(p), dtype=np.float32)
        print(f"=== {p} : {len(table):,} rays")
        _report(table, len(table))
        print()
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    c = sub.add_parser("from-cloud", help="convert a recorded point cloud into a table")
    c.add_argument("input", help="CSV or NPY with x, y, z in the sensor frame")
    c.add_argument("--out", required=True)
    c.add_argument("--time-column", type=int, default=None,
                   help="column holding a per-point timestamp; restores acquisition order")
    c.add_argument("--min-range", type=float, default=0.05,
                   help="drop points closer than this before taking a direction")
    c.set_defaults(func=from_cloud)

    m = sub.add_parser("from-mujoco-lidar", help="export the third-party bundled table")
    m.add_argument("--model", default="mid360")
    m.add_argument("--out", required=True)
    m.set_defaults(func=from_mujoco_lidar)

    i = sub.add_parser("inspect", help="report the envelope and ordering of tables")
    i.add_argument("tables", nargs="+")
    i.set_defaults(func=_inspect)

    args = ap.parse_args()
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
