# Livox Mid-360S 3D LiDAR Sensor Example

This example casts a 3D scan pattern into a MuJoCo scene and publishes the
returns as a `sensor_msgs/PointCloud2` PDU.

The goal is to make the `lidar_3d` sensor profile easy to understand from the
example code:

- the model contains a sensor on a short post, near and far objects, and two walls
- the config separates `spec`, `mjcf_binding` and `pdu_config`, like the 2D LiDAR
  profiles already in `config/sensors/lidar/`
- `LivoxMid360SSensor` is created in `livox-mid360s-hakoniwa-asset.py`
- the constructor applies the JSON profile: field of view, range gate, accuracy bands
- `LivoxMid360SSensor.scan()` casts one frame with `mj_multiRay`
- `read_point_cloud.py` receives the PDU and renders it with Open3D

## Files

```text
examples/sensors/livox_mid360s/
  README.md
  CMakeLists.txt
  livox-mid360s-hakoniwa-asset.cpp   C++ publisher, for the C++ simulator
  livox-mid360s-hakoniwa-asset.py
  read_point_cloud.py
  scene_view.py                    MuJoCo scene plus cloud, for the publisher
  point_colors.py                  range palette shared by both views

python/
  livox_mid360s_sensor.py          the sensor itself, no Hakoniwa dependency
  livox_scan_pattern_tool.py       build and inspect scan-pattern tables

models/sensors/lidar_3d/
  livox-mid360s-sample.xml          static, what the tests measure against
  livox-mid360s-moving-sample.xml   gravity on, things move

config/sensors/lidar/
  livox-mid360s.json               uniform pattern, needs no external data
  livox-mid360s-table.json         table pattern, needs a table you supply
  scan_patterns/
    README.md                      how to build a table; tables are not committed

config/sensors/schema/
  lidar-3d.schema.json

config/
  livox-mid360s-pdudef-compact.json   robot Mid360S -> the pdutypes below
  livox-mid360s-pdutypes.json         one 385,024 byte PointCloud2 channel
  assets/livox-mid360s-hakoniwa-asset.json   manifest the C++ asset reads
  assets/livox-mid360s-moving-asset.json     the same, on the moving scene
  endpoint/livox_mid360s_endpoint.json
  endpoint/comm/shm_livox_mid360s_comm.json

src/sensors/lidar/lidar_3d_sensor.cpp    the C++ sensor
include/sensors/lidar/lidar_3d_sensor.hpp
include/hakoniwa/pdu/converter/sensor_msgs/point_cloud2.hpp
include/hakoniwa/pdu/adapter/sensor_msgs/point_cloud2.hpp

```

## Sensor API

The sensor exists twice, once per language. The Python API is below;
`include/sensors/lidar/lidar_3d_sensor.hpp` is the C++ one.


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

## A Scene That Moves

`livox-mid360s-sample.xml` is deliberately static: it carries no joints at all,
because the blind-cone and occlusion tests measure against fixed positions.
Nothing in it moves however long the simulation runs, which makes it a poor way
to tell whether the asset is advancing physics.

`livox-mid360s-moving-sample.xml` is the same sensor on the same mount with
gravity switched on and three things that move:

- a pendulum whose arm starts horizontal, so it swings on gravity alone with no
  actuator and no initial velocity, sweeping an occlusion shadow across the wall
- a ball released at 5 m that falls through the field of view and settles
- a tower of three boxes, each offset far enough that its centre of mass sits
  past the edge of the one below, so it topples and comes apart

`box_static` and the walls do not move, so a moving return is obviously the
scene and not the sensor.

```bash
python3 examples/sensors/livox_mid360s/livox-mid360s-hakoniwa-asset.py --viewer \
    --scene models/sensors/lidar_3d/livox-mid360s-moving-sample.xml
```

The C++ asset takes a manifest rather than a scene, so it reads the moving one
through its own:

```bash
./src/cmake-build/examples/sensors/livox_mid360s/livox-mid360s-hakoniwa-asset \
    config/assets/livox-mid360s-moving-asset.json
```

## Sensor Config

`lidar_3d` is a new value for `spec.type`. Its sibling `lidar_2d` is what the
shipped 2D profiles use, `lds-01.json` and `urg-04lx-ug01.json` among them, and
`lidar_3d` keeps as much of it as still applies: distances in millimetres,
angles in degrees, the same `DistanceAccuracy` band structure.

Two things change. The 2D `AngleRange` becomes a solid-angle `FieldOfView` plus
a `ScanPattern`. And `mjcf_binding` gains `source_site`, because a 3D scan is
cast from a MuJoCo site rather than from a body origin.

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

Tables are not committed. See `config/sensors/lidar/scan_patterns/README.md`.

## Run

Publisher and reader are separate Hakoniwa assets. The publisher owns Conductor;
the reader must not start it. That takes three terminals.

Both default to what this repository ships, so run them from the repository root
with no arguments. `hakopy` and `hako-cmd` come from `hakoniwa-core-pro`; see the
top-level README for that install. The rest come from pip:

```bash
python3 -m pip install mujoco numpy hakoniwa-pdu open3d
```

`open3d` is only needed to draw. The sensor and the publisher do not use it, and
the reader's `--headless` mode still imports it, so skip the reader entirely if
you would rather not install it.

The Python publisher and the reader need no build. The C++ publisher does, and
it needs what every C++ target here needs: the submodules, and `hakoniwa-core-pro`
and `hakoniwa-pdu-endpoint` installed. `./doctor.bash` reports what is missing,
and the top-level README has the install steps.

```bash
git submodule update --init --recursive
cmake -S src -B src/cmake-build -DCMAKE_BUILD_TYPE=Release
cmake --build src/cmake-build --target livox-mid360s-hakoniwa-asset -j"$(nproc)"
```

`HAKONIWA_CORE_ROOT` and `HAKONIWA_PDU_ENDPOINT_ROOT` point CMake at those two
packages when they are not under `/usr/local/hakoniwa`.

Terminal A, the publisher. The C++ one reads the manifest in `config/assets/`:

```bash
./src/cmake-build/examples/sensors/livox_mid360s/livox-mid360s-hakoniwa-asset
```

or the Python one, which reads the profile and the scene directly:

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

## Two Views Of The Same Cloud

There are two windows, and which one answers a question depends on what the
question is.

`--viewer` on the publisher draws the returns onto the scene:

```bash
python3 examples/sensors/livox_mid360s/livox-mid360s-hakoniwa-asset.py --viewer
```

This is the only place the cloud and the geometry that produced it appear
together, which is what makes an occlusion shadow readable: the points stop, and
the object that stopped them is right there. Only the publisher can do it. The
reader receives the cloud alone, with no MuJoCo model to draw it against.

`read_point_cloud.py` shows the cloud as a receiver sees it. That is the honest
view of what a consumer downstream of the PDU actually gets, with no scene to
fill in the gaps. It is also the cloud-only window: run the publisher without
`--viewer` and only the reader's window opens.

`scene_view.py` re-poses the meshes from MjModel every frame, so a scene with
joints follows its physics. Building them once and leaving them put looks like
the sensor drifting away from its own geometry, which is a confusing way to
learn that the scene is moving.

Both windows are Open3D. MuJoCo's own passive viewer is the obvious way to draw
on a MuJoCo scene and was tried first, through several rounds: it rendered, but
would not reliably take mouse input on the machine this was developed on, so the
overlay moved to Open3D, which does. Nothing is lost. MuJoCo is used here as a
ray caster and a scene description, not as a renderer and not as a physics
engine; `mj_forward` and `mj_multiRay` are the only simulation calls, and
nothing steps. `scene_view.py` rebuilds the eleven scene primitives as Open3D
meshes once, taking each colour from its MJCF material, and only the cloud
changes after that.

`--viewer-decimate N` draws every Nth point, and `--viewer-point-size` sets the
drawn size in pixels.

`--viewer` also paces the run to the wall clock, and says so when it starts.
Without it the publisher runs as fast as the simulation allows: `hakopy.usleep`
advances simulation time and returns at once rather than waiting in real time,
and it does not release the GIL, so the timing loop spins and nothing else on
the process gets a turn. Sleeping the rest of each frame in real time fixes
that. A run watched this way takes as long as the simulated time it covers;
without `--viewer` nothing is paced.

Closing either window stops its asset. Closing the publisher's stops the
simulation with it, since the publisher owns Conductor.

## Two Publishers, One Reader

The sensor exists in C++ and in Python, and so does the publisher. The reader is
Python in both cases: it receives a PDU and does not care what wrote it. This is
the shape `color_camera` already uses, a C++ publisher with `read_camera.py`.

Both are here because they answer different needs. `src/sensors` is where the
C++ simulator main loop finds its sensors, so only a C++ `lidar_3d` can be
mounted on the C++ robot samples. The Python one needs no build, which is what
makes the standalone example and the Business Pack Recipe quick to iterate on.

```text
LiDAR3DSensor (C++)  ─┐
                      ├─→ Mid360S/point_cloud ─→ read_point_cloud.py
LivoxMid360SSensor (py) ┘
```

They read the same profile, including `mjcf_binding` and `pdu_config`, so the
mount, the range gate, the accuracy bands and the channel budget cannot drift
between them. Both step MuJoCo at the model's timestep and scan once per sensor
frame, and both stamp the cloud with MuJoCo's own clock, so a cloud published by
one is the same cloud the other would have published.

The two arrange that differently, and the difference is forced. The C++ asset
registers with Hakoniwa at the model's timestep and lets its update scheduler
decide when to scan. The Python asset registers at the sensor's frame period and
takes the model's timesteps inside it, because registering at the timestep needs
a conductor cycle that does not match it: matching them stalls the run outright
once a second asset joins, and leaving the cycle at 100 ms advances simulation
at a tenth of real time.

Both let a ray through the sensor's own mount rather than stopping on it.
`mj_multiRay`'s `bodyexclude` drops only the named body's own geoms, not its
descendants, so a mount carrying a bracket or a housing, which is what a sensor
on a robot has, would otherwise be seen by its own sensor. Rays that hit one are
re-cast from just past it, as `lidar_2d` does. Dropping them instead would lose
whatever stands behind the mount.
`models/sensors/lidar_3d/livox-mid360s-nested-mount-test.xml` is the fixture for
that, and it is deliberately harsh: the bracket sits right in front of the
sensor, so about an eighth of the rays need a second cast and the Python scan
costs 68 ms a frame against 11 ms on the sample scene. A real mount occludes far
fewer.

Two differences are deliberate:

- The C++ sensor refuses a `table` scan pattern rather than approximating it.
  Replaying a recorded table is implemented in Python, which can read the `.npy`
  the tooling produces.
- Only the Python publisher has `--viewer`. Drawing the scene is an example
  convenience, and the C++ asset is the one meant to be mounted on a robot.

A reader started before the publisher writes will see a channel that has been
created but never written, which reads back as zeros: not an empty read, and not
a valid PDU. It skips those and says so once, as `read_camera.py` does.

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
