# Sensor Examples

This directory contains small examples for Hakoniwa MuJoCo sensor components.

Shared example-only helpers live under:

```text
examples/sensors/common/
```

They cover common pieces such as minimal MuJoCo world loading and `i/k/j/l` freejoint movement.

## Ultrasonic

See:

```text
examples/sensors/ultrasonic/README.md
```

The ultrasonic example demonstrates:

- loading a minimal MuJoCo model
- binding a range sensor to a MuJoCo `site`
- moving the robot body interactively with `i/k/j/l`
- measuring range with `s`
- visualizing the measured ray in the MuJoCo viewer
- converting the internal ultrasonic frame to `sensor_msgs/Range`
- publishing `sensor_msgs/Range` through Hakoniwa PDU with `ultrasonic-hakoniwa-asset`
- reading the range PDU from Python with `read_range.py`

Run from the repository root:

```bash
./src/cmake-build/examples/sensors/ultrasonic/ultrasonic-example
```

## Livox Mid-360S 3D LiDAR

See:

```text
examples/sensors/livox_mid360s/README.md
```

The 3D LiDAR example demonstrates:

- loading a MuJoCo scene with near and far objects, a pole and two walls
- casting a 3D scan pattern with `mj_multiRay` from a MuJoCo `site`
- applying a datasheet field of view, range gate and per-band accuracy from JSON
- keeping the sensor's own mount out of the returns with `exclude_body`
- packing a scan into `sensor_msgs/PointCloud2` and publishing it through Hakoniwa PDU
- reading the cloud back and rendering it live with `read_point_cloud.py`

The publisher exists in C++ and in Python; the reader is Python either way,
because it receives a PDU and does not care what wrote it. Two Hakoniwa assets,
so run it from the repository root in three terminals:

```bash
./src/cmake-build/examples/sensors/livox_mid360s/livox-mid360s-hakoniwa-asset
python3 examples/sensors/livox_mid360s/read_point_cloud.py
hako-cmd start
```

## Color Camera

See:

```text
examples/sensors/color_camera/README.md
```

The color camera example demonstrates:

- loading a minimal MuJoCo scene with red / green / blue panels
- opening a MuJoCo viewer and capturing an RGB camera frame with `s` from the viewer or terminal
- moving the camera body with `i/k/j/l`
- printing sample RGB values from the captured image
- writing the result to a PNG file

Run from the repository root:

```bash
./src/cmake-build/examples/sensors/color_camera/color-camera-example
```
