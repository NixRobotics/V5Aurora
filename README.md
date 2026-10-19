# V5Aurora

## Log replay tool

[tools/log_replay.py](tools/log_replay.py) is a desktop viewer for the `.csv` logs captured by the `Logger` (e.g. `log/aurora_skills_3.csv` from `autonomous_skills()`). It only needs Python with tkinter (no extra packages).

```
python tools/log_replay.py log/aurora_skills_3.csv
```

- Field view (NED: North/+X up, East/+Y right) with the robot pose, the path, and distance sensor beams (dashed = stale "last valid" value)
- Field zoom: mouse wheel at the pointer (1x to 16x), left-drag to pan,
  or use the **+ / -** buttons to zoom at the centre. **Fit field** resets
  zoom and pan. The view is preserved while stepping, playing, and resizing;
  paths, sensor rays, and diagnostic overlays all use the same view.
- Readout of every signal at the current sample, plus derived values (speed, turn rate, motor rpm, tracking wheel mm) and the X/Y position implied by each distance sensor vs. odometry
- Strip charts with a time cursor; pick the signal group per chart and click/drag to seek
- Play/pause at 0.1x–8x, or single-step: Space, ←/→ (1 sample), Shift (10), Ctrl (50), Home/End
- Playback follows `sys_time`, so 10 ms, 20 ms or uneven sample rates all work

### Distance status logs

The optional `status` column (for example in `log/log_new_format.csv`) packs four
positive 4-bit codes, high to low: **b1, b2, left, right**. The viewer decodes these
in the sensor readouts and field overlay; rejected/fault/disabled beams are dashed.
The distance values remain the **last valid readings**, not the rejected measurements.

Codes: 0 normal, 1 disconnected, 2 no object, 3 out of range, 4 sensor value error,
5 no new timestamp, 6 unexpected location, 7 angle filtered, 8 location filtered,
9 disabled. Unknown nibble codes are shown explicitly.

Code **5 is neutral** for beam styling and maps to 0 in the optional
**Distance status (5 ignored)** chart. It does not cause stale warnings.
Status and readings may be captured at different points in the odometry/logging
cycles (10 ms vs. 20 ms in the sample). They are shown as logged, without
inferring exact acceptance times, shifting readings, or carrying faults forward.
Sensor-implied positions are held-value estimates, not evidence of acceptance.
Logs without status retain the original unchanged-value age/dashed-beam behavior;
that age is time since a numeric change, not a measured sensor-update timestamp.

### Offline tracking diagnostics

Use **Diagnostics > Tracking comparison layout** for:

- **Teal tracking-only / magenta motor-only paths**, using the logged gyro and
  initial X/Y, without distance corrections. Blue remains the logged fused pose.
  The path overlays can be toggled separately in the Diagnostics menu.
- Rotation-compensated forward/right tracking velocity versus X-drive encoder
  velocity, trailing approximately 200 ms; tracking-minus-motor residuals and
  accumulated path separation. Differences indicate disagreement, not which
  sensor is wrong. Both reconstructions share the gyro.

The desktop model uses 156 mm tracking circumferences, forward-wheel right offset
22.5 mm, side-wheel forward offset 59.6 mm, motor gearing 24/48, drive circumference
220 mm, forward efficiency 1/1.045, and ideal 45-degree drive mixing. These
assumptions are constants near the top of `tools/log_replay.py`; they are **not**
automatically read from robot code or fitted to each log. Reconstruction uses
midpoint-heading constant-twist integration, not a replay of the robot filter.
Required missing channels leave the corresponding diagnostics unavailable.

**Diagnostics > Distance timing layout** adds dots for intervals between observed
distance-value changes, alongside distances, status and motion disagreement.
Intervals appear only at numeric changes; faults/filters break the sequence and
status 5 remains neutral. These are **not sensor update intervals or processing
latency measurements**: identical fresh readings and missed intermediate samples
cannot be distinguished in the existing logs. No lag compensation, contact-loss
classification, PID tuning or robot code changes are performed.

Model assumptions are also available in **Diagnostics > Model assumptions /
limitations**. Derived values appear in the readout's Diagnostics group.

Run the desktop regression tests with:

```
python -m unittest discover -s tools -p test_log_replay.py -v
```
