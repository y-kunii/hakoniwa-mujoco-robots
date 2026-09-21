# Examples

This directory contains small, user-facing examples for trying individual Hakoniwa MuJoCo features without running the full robot demos.

## Sensor Examples

Sensor examples live under:

```text
examples/sensors/
```

Available examples:

- [Ultrasonic Sensor Example](sensors/ultrasonic/README.md)
  - minimal MuJoCo ultrasonic range sensor
  - interactive keyboard movement
  - measured ray visualization in the MuJoCo viewer
  - `sensor_msgs/Range` PDU conversion path
  - Hakoniwa PDU publisher and Python reader examples
- [Color Camera Sensor Example](sensors/color_camera/README.md)
  - minimal MuJoCo RGB camera sensor
  - red / green / blue panels in a small scene
  - MuJoCo viewer with `s` key PNG capture from the viewer or terminal
  - `i/k/j/l` camera movement
  - left / center / right RGB sample printout
  - PNG output for quick visual inspection
- [Livox Mid-360S 3D LiDAR Example](sensors/livox_mid360s/README.md)
  - 3D scan pattern cast with `mj_multiRay` from a MuJoCo site
  - datasheet field of view, range gate and per-band accuracy from JSON
  - near and far objects, a pole and two walls for occlusion shadows
  - `sensor_msgs/PointCloud2` PDU publisher and Python reader
  - live Open3D rendering of the received cloud
  - Python only, so no CMake build is needed

These examples are intended to complement the larger TurtleBot3 and forklift demos described in the top-level README.

## Actuator Examples

Actuator examples live under:

```text
examples/actuators/
```

Available examples:

- [Joint Actuator Example](actuators/joint/README.md)
  - minimal MuJoCo `<position>` actuator
  - minimal MuJoCo `<velocity>` actuator
  - MuJoCo viewer with keyboard target control
  - JSON config loaded through `JointActuatorImpl`
  - `SetTarget()` writes targets to MuJoCo `ctrl[]`
  - Hakoniwa PDU command receiver and Python command sender examples
