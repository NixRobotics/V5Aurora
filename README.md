# V5Aurora

## Log replay tool

[tools/log_replay.py](tools/log_replay.py) is a desktop viewer for the `.csv` logs captured by the `Logger` (e.g. `log/aurora_skills_3.csv` from `autonomous_skills()`). It only needs Python with tkinter (no extra packages).

```
python tools/log_replay.py log/aurora_skills_3.csv
```

- Field view (NED: North/+X up, East/+Y right) with the robot pose, the path, and distance sensor beams (dashed = stale "last valid" value)
- Readout of every signal at the current sample, plus derived values (speed, turn rate, motor rpm, tracking wheel mm) and the X/Y position implied by each distance sensor vs. odometry
- Strip charts with a time cursor; pick the signal group per chart and click/drag to seek
- Play/pause at 0.1x–8x, or single-step: Space, ←/→ (1 sample), Shift (10), Ctrl (50), Home/End
- Playback follows `sys_time`, so 10 ms, 20 ms or uneven sample rates all work

