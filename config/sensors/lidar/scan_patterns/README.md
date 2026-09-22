# Scan-pattern tables

A ray-angle table is an `(N, 2)` float32 array of `(azimuth, elevation)` in
radians, stored in **acquisition order**. `lidar_3d` profiles with
`ScanPattern.Type: table` replay it, wrapping when it runs out, which is what
reproduces non-repetitive scanning: consecutive frames use different ray
directions, so accumulated coverage keeps improving with observation time.

**Tables are not committed here.** They are data you supply. Generate one with
`python/livox_scan_pattern_tool.py`.

## From your own sensor (the only measured source)

A recorded point cloud *is* a scan-pattern measurement: each point's direction
is the ray that produced it.

```bash
python3 python/livox_scan_pattern_tool.py from-cloud recording.csv \
    --out config/sensors/lidar/scan_patterns/livox-mid360s.npy --time-column 3
```

Record in a space with reflective surfaces in most directions - rays that hit
nothing produce no point and leave holes in the table. Keep the sensor static,
or its motion is baked into the pattern. Record longer than the pattern's own
repeat period, and note which motor mode you used: the Mid-360S adds a Motor
Slow Mode that changes the scan trajectory.

Then state where it came from:

```json
"TableProvenance": "Measured from recording.csv on 2026-10-01, serial <...>, default motor mode, static mount."
```

## From the third-party mujoco-lidar package

```bash
python3 python/livox_scan_pattern_tool.py from-mujoco-lidar --model mid360 \
    --out config/sensors/lidar/scan_patterns/livox-mid360s.npy
```

That package ships its tables with no statement of how they were produced. A
hardware recording, an SDK export and a synthesised model are all consistent
with what is published, so results from it are a hypothesis, not a measurement.
The tool says so and suggests wording for `TableProvenance`.

## Checking a table

```bash
python3 python/livox_scan_pattern_tool.py inspect config/sensors/lidar/scan_patterns/livox-mid360s.npy
```

Reports the azimuth and elevation envelope, whether entries are in acquisition
order, and whether the lowest available elevation varies with azimuth. The last
one matters: if it does, the floor blind zone is an elongated lobe rather than
the circle a single published vertical FOV implies.

## No table, no problem

`ScanPattern.Type: uniform` needs no table. It samples the declared field of
view uniformly and depends on nothing but the datasheet, at the cost of not
reproducing non-repetitive accumulation. `config/sensors/lidar/livox-mid360s.json`
uses it.
