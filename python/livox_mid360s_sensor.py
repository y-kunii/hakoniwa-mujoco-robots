#!/usr/bin/env python3
"""Livox Mid-360S 3D LiDAR sensor for Hakoniwa MuJoCo robots.

Casts a 3D scan pattern into a MuJoCo scene with ``mj_multiRay`` and returns the
returns as points in the sensor frame, ready to be packed into a
``sensor_msgs/PointCloud2``.

The profile is read from a ``lidar-3d.schema.json`` config, so field of view,
range gate, accuracy bands and scan pattern come from the JSON rather than from
code. Two scan patterns are supported:

``uniform``
    Derived from the declared field of view alone: azimuth and elevation are
    sampled uniformly inside the published envelope. It depends on nothing but
    the datasheet, and its floor blind zone is a true circle.

``table``
    Replays a recorded ray-angle table in acquisition order, wrapping when it
    runs out. This is what reproduces non-repetitive scanning, where accumulated
    coverage keeps improving as observation time grows. The table is a file you
    supply; state where it came from in ``ScanPattern.TableProvenance``.

This module depends only on ``mujoco`` and ``numpy``. It knows nothing about
Hakoniwa, so it can be exercised without a running simulation:

    python3 python/livox_mid360s_sensor.py \\
        --config config/sensors/lidar/livox-mid360s.json \\
        --scene models/livox_mid360s/scene.xml
"""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np

MM_PER_M = 1000.0


@dataclass(frozen=True)
class Scan:
    """One frame of returns, in the sensor frame."""

    points: np.ndarray  # (N, 3) float32
    distances: np.ndarray  # (N,) float32, metres
    geom_ids: np.ndarray  # (N,) int32, MuJoCo geom that returned the hit
    rays_cast: int  # rays emitted, including non-returns

    @property
    def hit_ratio(self) -> float:
        return len(self.distances) / self.rays_cast if self.rays_cast else 0.0


class ScanPattern:
    """Ray directions for one frame, in the sensor frame."""

    def __init__(self, spec: dict, config_dir: Path, seed: int = 0) -> None:
        pattern = spec["ScanPattern"]
        fov = spec["FieldOfView"]
        self.kind: str = pattern["Type"]
        self.point_rate: float = float(pattern["PointRate"])
        self.frame_rate: float = float(pattern["FrameRate"])
        self.samples: int = max(1, int(round(self.point_rate / self.frame_rate)))

        self.az_min = math.radians(float(fov["Horizontal"]["Min"]))
        self.az_max = math.radians(float(fov["Horizontal"]["Max"]))
        self.el_min = math.radians(float(fov["Vertical"]["Min"]))
        self.el_max = math.radians(float(fov["Vertical"]["Max"]))

        self._rng = np.random.default_rng(seed)
        self._table: np.ndarray | None = None
        self._cursor = 0

        if self.kind == "table":
            self._table = self._load_table(pattern, config_dir)

    @staticmethod
    def _load_table(pattern: dict, config_dir: Path) -> np.ndarray:
        path = (config_dir / pattern["TableFile"]).resolve()
        if not path.exists():
            raise FileNotFoundError(
                f"ScanPattern.TableFile not found: {path}\n"
                "A 'table' pattern replays a ray-angle table you supply. Either point "
                "TableFile at one, or switch ScanPattern.Type to 'uniform', which needs "
                "no external data."
            )
        fmt = pattern["TableFormat"]
        if fmt == "npy_theta_phi_rad":
            table = np.load(path)
        elif fmt == "csv_theta_phi_deg":
            table = np.radians(np.loadtxt(path, delimiter=",", dtype=np.float64))
        else:  # pragma: no cover - schema restricts the enum
            raise ValueError(f"unsupported TableFormat: {fmt}")
        table = np.asarray(table, dtype=np.float32)
        if table.ndim != 2 or table.shape[1] != 2:
            raise ValueError(
                f"{path}: expected an (N, 2) array of (azimuth, elevation), got {table.shape}"
            )
        return table

    @property
    def table_rays(self) -> int:
        return 0 if self._table is None else len(self._table)

    def next_frame(self) -> tuple[np.ndarray, np.ndarray]:
        """Azimuth and elevation in radians for the next frame."""
        if self._table is None:
            az = self._rng.uniform(self.az_min, self.az_max, self.samples)
            el = self._rng.uniform(self.el_min, self.el_max, self.samples)
            return az, el

        n, total = self.samples, len(self._table)
        idx = (self._cursor + np.arange(n)) % total
        self._cursor = (self._cursor + n) % total
        rows = self._table[idx]
        return rows[:, 0], rows[:, 1]


class AccuracyModel:
    """Range noise from the profile's DistanceAccuracy bands."""

    def __init__(self, bands: list, seed: int = 0) -> None:
        self._rng = np.random.default_rng(seed + 1)
        self._bands = []
        for band in bands:
            lo = float(band["Range"]["Min"]) / MM_PER_M
            hi = float(band["Range"]["Max"]) / MM_PER_M
            if band["Type"] == "independent":
                acc = band["DistanceIndependentAccuracy"]
                self._bands.append((lo, hi, "abs", float(acc["StdDev"])))
            else:
                acc = band["DistanceDependentAccuracy"]
                self._bands.append((lo, hi, "pct", float(acc["Percentage"]) / 100.0))

    def apply(self, distances: np.ndarray) -> np.ndarray:
        """Return distances perturbed by the band that covers each one."""
        if not self._bands:
            return distances
        sigma = np.zeros_like(distances)
        for lo, hi, mode, value in self._bands:
            m = (distances >= lo) & (distances < hi)
            if not np.any(m):
                continue
            sigma[m] = value if mode == "abs" else distances[m] * value
        noisy = distances + self._rng.normal(0.0, 1.0, distances.shape) * sigma
        return np.maximum(noisy, 0.0).astype(distances.dtype)


class LivoxMid360SSensor:
    """3D LiDAR bound to an MJCF body or site."""

    def __init__(
        self,
        model: mujoco.MjModel,
        config_path: str | Path,
        *,
        seed: int = 0,
        apply_noise: bool = True,
    ) -> None:
        config_path = Path(config_path).resolve()
        config = json.loads(config_path.read_text(encoding="utf-8"))
        spec = config["spec"]
        binding = config["mjcf_binding"]

        if spec["type"] != "lidar_3d":
            raise ValueError(f"{config_path}: expected spec.type lidar_3d, got {spec['type']}")

        self.model = model
        self.config_path = config_path
        self.name: str = spec["name"]
        self.frame_id: str = binding.get("frame_id_override", spec["frame_id"])
        self.pdu = config["pdu_config"]

        self.min_range = float(spec["DetectionDistance"]["Min"]) / MM_PER_M
        self.max_range = float(spec["DetectionDistance"]["Max"]) / MM_PER_M

        self.pattern = ScanPattern(spec, config_path.parent, seed=seed)
        self.noise = AccuracyModel(spec["DistanceAccuracy"], seed=seed) if apply_noise else None

        # lidar-2d binds to a body; source_site is an optional finer origin.
        self._site_id = -1
        site_name = binding.get("source_site")
        if site_name:
            self._site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site_name)
            if self._site_id < 0:
                raise ValueError(f"mjcf_binding.source_site not in model: {site_name}")

        self._body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, binding["source_body"])
        if self._body_id < 0:
            raise ValueError(f"mjcf_binding.source_body not in model: {binding['source_body']}")

        self.body_exclude = -1
        exclude = binding.get("exclude_body")
        if exclude:
            self.body_exclude = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, exclude)
            if self.body_exclude < 0:
                raise ValueError(f"mjcf_binding.exclude_body not in model: {exclude}")

    @property
    def samples_per_frame(self) -> int:
        return self.pattern.samples

    def origin(self, data: mujoco.MjData) -> tuple[np.ndarray, np.ndarray]:
        """Ray origin and orientation: the site when given, else the body."""
        if self._site_id >= 0:
            return (np.asarray(data.site(self._site_id).xpos, dtype=np.float64),
                    np.asarray(data.site(self._site_id).xmat, dtype=np.float64).reshape(3, 3))
        return (np.asarray(data.xpos[self._body_id], dtype=np.float64),
                np.asarray(data.xmat[self._body_id], dtype=np.float64).reshape(3, 3))

    def scan(self, data: mujoco.MjData) -> Scan:
        """Cast one frame. Advances the scan pattern."""
        az, el = self.pattern.next_frame()
        nray = az.shape[0]

        cos_el = np.cos(el)
        local = np.stack((cos_el * np.cos(az), cos_el * np.sin(az), np.sin(el)),
                         axis=-1).astype(np.float64)

        pos, rot = self.origin(data)
        world = local @ rot.T
        world /= np.linalg.norm(world, axis=1, keepdims=True)

        dist = np.full(nray, self.max_range, dtype=np.float64)
        geomid = np.full(nray, 0, dtype=np.int32)
        mujoco.mj_multiRay(
            m=self.model, d=data,
            pnt=pos.reshape(3, 1), vec=world.flatten(),
            geomgroup=None, flg_static=1, bodyexclude=self.body_exclude,
            geomid=geomid, dist=dist, normal=None,
            nray=nray, cutoff=self.max_range,
        )

        # geomid < 0 marks a non-return; the range gate drops the rest.
        valid = (geomid >= 0) & (dist > self.min_range) & (dist < self.max_range)
        d = dist[valid]
        if self.noise is not None and len(d):
            d = self.noise.apply(d)

        return Scan(
            points=(local[valid] * d[:, None]).astype(np.float32),
            distances=d.astype(np.float32),
            geom_ids=geomid[valid].copy(),
            rays_cast=nray,
        )


def _main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", required=True, help="lidar-3d profile JSON")
    ap.add_argument("--scene", required=True, help="MJCF scene containing the sensor binding")
    ap.add_argument("--frames", type=int, default=5)
    ap.add_argument("--no-noise", action="store_true", help="disable the DistanceAccuracy model")
    args = ap.parse_args()

    model = mujoco.MjModel.from_xml_path(args.scene)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    sensor = LivoxMid360SSensor(model, args.config, apply_noise=not args.no_noise)

    print(f"profile     : {sensor.name}  ({sensor.config_path.name})")
    print(f"frame_id    : {sensor.frame_id}")
    print(f"range gate  : {sensor.min_range:.2f} .. {sensor.max_range:.1f} m")
    print(f"pattern     : {sensor.pattern.kind}  {sensor.samples_per_frame:,} rays/frame"
          + (f"  table={sensor.pattern.table_rays:,} rays" if sensor.pattern.table_rays else ""))
    print(f"noise       : {'on' if sensor.noise else 'off'}")
    print(f"pdu         : {sensor.pdu['pdu_name']}  {sensor.pdu['message_type']}")
    print()
    print(f"{'frame':>6}{'returns':>10}{'hit %':>8}{'min m':>8}{'max m':>9}")
    print("-" * 41)
    for i in range(1, args.frames + 1):
        s = sensor.scan(data)
        print(f"{i:>6}{len(s.distances):>10,}{100*s.hit_ratio:>7.1f}%"
              f"{s.distances.min():>8.2f}{s.distances.max():>9.2f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(_main())
