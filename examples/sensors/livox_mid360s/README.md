# Livox Mid-360S 3D LiDAR Sensor Example

This example casts a 3D scan pattern into a MuJoCo scene and publishes the
returns as a `sensor_msgs/PointCloud2` PDU.

The goal is to make the `lidar_3d` sensor profile easy to understand from the
example code:

- the model contains a sensor on a short post, near and far objects, and two walls
- the config separates `spec`, `mjcf_binding` and `pdu_config`, as `lidar_2d` does
- `LivoxMid360SSensor` is created in `livox-mid360s-hakoniwa-asset.py`
- the constructor applies the JSON profile: field of view, range gate, accuracy bands
- `LivoxMid360SSensor.scan()` casts one frame with `mj_multiRay`
- `read_point_cloud.py` receives the PDU and renders it with Open3D

## Files

```text
examples/sensors/livox_mid360s/
  README.md
  livox-mid360s-hakoniwa-asset.py
  read_point_cloud.py

python/
  livox_mid360s_sensor.py          the sensor itself, no Hakoniwa dependency
  livox_scan_pattern_tool.py       build and inspect scan-pattern tables

models/sensors/lidar_3d/
  livox-mid360s-sample.xml

config/sensors/lidar/
  livox-mid360s.json               uniform pattern, needs no external data
  livox-mid360s-table.json         table pattern, needs a table you supply

config/sensors/schema/
  lidar-3d.schema.json

config/
  livox-mid360s-pdudef-compact.json   robot Mid360S -> the pdutypes below
  livox-mid360s-pdutypes.json         one 385,024 byte PointCloud2 channel

scan_patterns/
  README.md                        how to build a table; tables are not committed
```

## Sensor API

```python
from livox_mid360s_sensor import LivoxMid360SSensor

sensor = LivoxMid360SSensor(model, "config/sensors/lidar/livox-mid360s.json")
scan = sensor.scan(data)
scan.points      # (N, 3) float32, sensor frame
scan.distances   # (N,) float32, metres
scan.geom_ids    # (N,) int32, which geom returned each point
scan.rays_cast   # rays emitted, including non-returns
```

The sensor depends only on `mujoco` and `numpy`. It can be exercised without a
running simulation:

```bash
python3 python/livox_mid360s_sensor.py \
    --config config/sensors/lidar/livox-mid360s.json \
    --scene models/sensors/lidar_3d/livox-mid360s-sample.xml
```

## Model

`models/sensors/lidar_3d/livox-mid360s-sample.xml` places the sensor 0.5 m above
the floor on a post. The whole mount is the profile's `exclude_body`, so the
sensor does not detect itself, and `lidar_site` is its `source_site`.

The objects are chosen to exercise the published field of view. At 0.5 m
mounting height the datasheet's -7 degree lower limit puts the nearest floor
return at `0.5 / tan(7 deg)` = 4.07 m, so `box_low_near` at 1.2 m returns
nothing while `box_tall_near` at the same distance is seen. The pole and walls
produce occlusion shadows.

## Sensor Config

A `lidar_3d` profile follows `lidar_2d`: distances in millimetres, angles in
degrees, the same `DistanceAccuracy` bands, the same `mjcf_binding`. It replaces
the 2D `AngleRange` with a solid-angle `FieldOfView` plus a `ScanPattern`.

```json
"FieldOfView": {
  "Horizontal": { "Min": -180.0, "Max": 180.0 },
  "Vertical":   { "Min": -7.0,   "Max": 52.0 }
},
"ScanPattern": {
  "Type": "uniform",
  "PointRate": 200000,
  "FrameRate": 10.0
}
```

Rays per frame is `PointRate / FrameRate`, so the value is checkable against the
datasheet rather than hard-coded.

`ScanPattern.Type` selects how rays are distributed:

- `uniform` samples the declared field of view and depends on nothing but the
  datasheet. Its floor blind zone is a true circle.
- `table` replays a recorded ray-angle sequence in acquisition order, which is
  what reproduces non-repetitive scanning: consecutive frames use different
  directions so accumulated coverage keeps improving with observation time.

Tables are not committed. See `scan_patterns/README.md`.

## Run

Publisher and reader are separate Hakoniwa assets. The publisher owns Conductor;
the reader must not start it. Use three terminals, as `color_camera` does.

Both default to what this repository ships, so run them from the repository root
with no arguments. `hakopy` and `hako-cmd` come from `hakoniwa-core-pro`; see the
top-level README for that install. The rest come from pip:

```bash
python3 -m pip install mujoco numpy hakoniwa-pdu open3d
```

`open3d` is only needed to draw. The sensor and the publisher do not use it, and
the reader's `--headless` mode still imports it, so skip the reader entirely if
you would rather not install it.

Terminal A, the publisher:

```bash
python3 examples/sensors/livox_mid360s/livox-mid360s-hakoniwa-asset.py
```

Terminal B, the reader (add `--headless` where there is no display):

```bash
python3 examples/sensors/livox_mid360s/read_point_cloud.py
```

Terminal C, once both report `WAITING`:

```bash
hako-cmd start
```

Paths can be overridden. A composition that sizes the channel itself, such as a
Hakoniwa Business Pack Recipe, passes its own `--config` and `--pdu-size`:

```bash
python3 examples/sensors/livox_mid360s/livox-mid360s-hakoniwa-asset.py \
    --config <pdudef.json> \
    --profile config/sensors/lidar/livox-mid360s.json \
    --scene models/sensors/lidar_3d/livox-mid360s-sample.xml \
    --pdu-size 385024
```

## Example Output

```text
sensor root : /path/to/hakoniwa-mujoco-robots
profile     : livox_mid360s  pattern=uniform
channel     : Mid360S/point_cloud  pdu_size=385,024
capacity    : 24,016 points at point_step 16
INFO: registered. WAITING for hako-cmd start
      100000 us  frame    1   5,070 pts   81,880 B
```

## Hakoniwa PDU Publisher / Reader

The PDU for `sensor_msgs/PointCloud2` is fixed size, so the channel must be
sized before anything runs. The serialised envelope is a constant 760 bytes
regardless of point count, so:

```text
pdu_size = 760 + max_points * point_step
```

`pdu_config.max_points` and `pdu_config.point_step` in the profile are what a
publisher budgets against. A frame that exceeds the budget is truncated rather
than overrunning the channel.

The publisher does not restate that size. With no `--pdu-size` it reads the
channel its `--config` pdudef declares, so the value cannot drift from the
channel the runtime actually opens. `tests/test_lidar_3d_profile.py` checks the
shipped pdutypes against the profile.

`read_point_cloud.py` draws from inside the simulation callback, on the main
thread, rather than splitting the GUI and `hakopy.start()` across two threads as
`read_camera.py` does. That split works for cv2, which needs only a brief slice
per frame; Open3D needs continuous time, and `hakopy.start()` holds the GIL
between callbacks, so the render loop was starved to about one frame per second.
Drawing inside the callback means a slow render slows simulation time instead.
`--render-every N` trades displayed frames back for cadence.

## Notes

- Only the default motor mode is simulated. The Mid-360S adds a Motor Slow Mode
  that changes the scan trajectory and has no Mid-360 equivalent.
- Reflectivity, return tag and per-point timestamps are not produced. Range
  noise is, from the profile's `DistanceAccuracy` bands.
- The floor blind zone grows linearly with mounting height: `8.1 * height` from
  the datasheet's -7 degree limit. Plan with that figure. A `table` pattern may
  show an azimuth-dependent lobe reaching further, but unless the table came
  from hardware you can account for, that lobe is a hypothesis.
