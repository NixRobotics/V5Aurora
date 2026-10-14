#!/usr/bin/env python3
"""
Aurora log replay - desktop viewer for the .csv logs written by the V5PythonFramework Logger.

Usage:
    python tools/log_replay.py [path/to/log.csv]

Expected columns (as written by autonomous_skills() in src/main.py):
    sys_time, lfm_value, lbm_value, rfm_value, rbm_value, gyro_value, fwd_value, side_value, y, x, b1, b2, l, r

Units / conventions (match src/main.py and the Logger):
    - sys_time: brain system time (ms). Playback is driven by this column, so any sample rate works.
    - motors: position in TURNS
    - gyro: inertial rotation in DEGREES (unwrapped, clockwise positive)
    - fwd/side: rotation sensor position in DEGREES
    - x, y: odometry in mm, NED frame: X = North (robot forward at heading 0), Y = East (robot right)
    - b1, b2, l, r: last *valid* distance readings in mm (-1 = never valid)

Only uses the Python standard library (tkinter).
"""

import bisect
import csv
import math
import os
import sys
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

# ------------------------------------------------------------------------------ #
# Robot / field geometry (mirrors constants in src/main.py)
# ------------------------------------------------------------------------------ #

FIELD_MM = 3600.0
TILE_MM = 600.0
ROBOT_LENGTH = 183 * 2.0
ROBOT_WIDTH = 192 * 2.0
HIDDEN_PERIMETER = 15.0
BACK_DISTANCE1_FROM_BACK = 75.0
BACK_DISTANCE2_FROM_BACK = 69.0
BACK_DISTANCE_SEPARATION = 14 * 25.4
LEFT_DISTANCE_FROM_LEFT = 5.0
RIGHT_DISTANCE_FROM_RIGHT = 5.0
TRACKING_WHEEL_MM = 156.0
DRIVE_GEAR_RATIO = 24 / 48
DRIVE_WHEEL_MM = 220.0

# Distance sensor mounting in robot body frame:
# key -> (forward offset mm, right offset mm, beam direction deg clockwise from robot forward, color, label)
SENSORS = {
    "b1": (-(ROBOT_LENGTH / 2 - BACK_DISTANCE1_FROM_BACK), -BACK_DISTANCE_SEPARATION / 2, 180.0, "#d62728", "Back 1 (left)"),
    "b2": (-(ROBOT_LENGTH / 2 - BACK_DISTANCE2_FROM_BACK), BACK_DISTANCE_SEPARATION / 2, 180.0, "#ff7f0e", "Back 2 (right)"),
    "l": (0.0, -(ROBOT_WIDTH / 2 - LEFT_DISTANCE_FROM_LEFT), -90.0, "#2ca02c", "Left"),
    "r": (0.0, ROBOT_WIDTH / 2 - RIGHT_DISTANCE_FROM_RIGHT, 90.0, "#1f77b4", "Right"),
}

# A "last valid" distance value that hasn't changed for this long is drawn dimmed (stale)
STALE_MS = 150
# Velocities / rpm are central differences over +/- this many ms (independent of log sample rate)
RATE_HALF_WINDOW_MS = 40
# Sample period assumed only if the log has no time column at all
DEFAULT_SAMPLE_MS = 10

COMPASS = ["N", "E", "S", "W"]

MOTORS = [("lfm", "Left front"), ("lbm", "Left back"), ("rfm", "Right front"), ("rbm", "Right back")]

# Chart groups: name -> list of (signal key, label, color)
CHART_GROUPS = {
    "Heading (deg)": [("gyro", "Gyro", "#9467bd")],
    "Position (mm)": [("x", "X (N)", "#d62728"), ("y", "Y (E)", "#1f77b4")],
    "Distance sensors (mm)": [(k, v[4], v[3]) for k, v in SENSORS.items()],
    "Drive motors (rpm)": [("lfm_rpm", "LF", "#d62728"), ("lbm_rpm", "LB", "#ff7f0e"),
                           ("rfm_rpm", "RF", "#1f77b4"), ("rbm_rpm", "RB", "#17becf")],
    "Speed (mm/s)": [("speed", "Speed", "#000000"), ("vn", "vN", "#d62728"), ("ve", "vE", "#1f77b4")],
    "Turn rate (deg/s)": [("turn_rate", "Turn rate", "#9467bd")],
    "Tracking wheels (mm)": [("fwd_mm", "Forward", "#d62728"), ("side_mm", "Side", "#1f77b4")],
    "Drive motors (turns)": [("lfm", "LF", "#d62728"), ("lbm", "LB", "#ff7f0e"),
                             ("rfm", "RF", "#1f77b4"), ("rbm", "RB", "#17becf")],
    "Sample interval (ms)": [("dt", "dt", "#555555")],
}
DEFAULT_CHARTS = ["Heading (deg)", "Position (mm)", "Distance sensors (mm)", "Drive motors (rpm)"]

CHART_WINDOWS = {"Full log": None, "30 s": 30000, "10 s": 10000, "5 s": 5000, "2 s": 2000}
SPEEDS = ["0.1x", "0.25x", "0.5x", "1x", "2x", "4x", "8x"]


def compass_index(heading):
    return int(((heading % 360.0) + 45.0) % 360.0 // 90.0)


def angle_in_tolerance(heading, tol):
    h = heading % 360.0
    return min(h % 90.0, 90.0 - (h % 90.0)) < tol


def body_to_field(x, y, theta_deg, fwd, right):
    """Robot body offset (forward, right) -> field NED (north, east)."""
    t = math.radians(theta_deg)
    return (x + fwd * math.cos(t) - right * math.sin(t),
            y + fwd * math.sin(t) + right * math.cos(t))


# ------------------------------------------------------------------------------ #
# Log data
# ------------------------------------------------------------------------------ #

class LogData:
    def __init__(self, path):
        self.path = path
        with open(path, newline="") as f:
            rows = [r for r in csv.reader(f) if r and any(c.strip() for c in r)]
        if len(rows) < 2:
            raise ValueError("Log file has no data rows")

        header = [self._normalize(h) for h in rows[0]]
        cols = {h: [] for h in header}
        for r in rows[1:]:
            if len(r) < len(header):
                continue
            try:
                vals = [float(v) for v in r[:len(header)]]
            except ValueError:
                continue
            for h, v in zip(header, vals):
                cols[h].append(v)

        self.n = len(next(iter(cols.values())))
        if self.n == 0:
            raise ValueError("Log file has no numeric data rows")
        self.cols = cols
        self.header = header

        # Time base: sys_time, else device timestamps (Logger include_device_times mode), else assumed rate
        time_col = "sys_time" if "sys_time" in cols else next((h for h in header if h.endswith("_time")), None)
        raw_t = cols[time_col] if time_col else [i * DEFAULT_SAMPLE_MS for i in range(self.n)]
        self.time_source = time_col or "assumed {} ms".format(DEFAULT_SAMPLE_MS)
        t0 = raw_t[0]
        # guarantee monotonic time so bisect-based seeking works even if timestamps glitch
        self.t = []
        last = 0.0
        for v in raw_t:
            last = max(last, v - t0)
            self.t.append(last)
        self.duration = self.t[-1]
        diffs = [b - a for a, b in zip(self.t, self.t[1:])]
        self.median_dt = sorted(diffs)[len(diffs) // 2] if diffs else 0.0

        self._derive()

    @staticmethod
    def _normalize(h):
        h = h.strip().lower()
        return h[:-6] if h.endswith("_value") else h

    def has(self, key):
        return key in self.cols

    def get(self, key, i):
        c = self.cols.get(key)
        return None if c is None else c[i]

    def _central_rate(self, values, scale=1000.0):
        """Central difference rate (per second * scale/1000) over +/- RATE_HALF_WINDOW_MS."""
        n, t = self.n, self.t
        out = [0.0] * n
        a = b = 0
        for i in range(n):
            while t[i] - t[a] > RATE_HALF_WINDOW_MS:
                a += 1
            while b < n - 1 and t[b + 1] - t[i] <= RATE_HALF_WINDOW_MS:
                b += 1
            lo, hi = (a, b) if b > a else (max(0, i - 1), min(n - 1, i + 1))
            dt = t[hi] - t[lo]
            out[i] = (values[hi] - values[lo]) / dt * scale if dt > 0 else 0.0
        return out

    def _derive(self):
        c = self.cols
        c["dt"] = [0.0] + [b - a for a, b in zip(self.t, self.t[1:])]
        if "x" in c and "y" in c:
            c["vn"] = self._central_rate(c["x"])
            c["ve"] = self._central_rate(c["y"])
            c["speed"] = [math.hypot(a, b) for a, b in zip(c["vn"], c["ve"])]
        if "gyro" in c:
            c["turn_rate"] = self._central_rate(c["gyro"])
        for key, _ in MOTORS:
            if key in c:
                c[key + "_rpm"] = self._central_rate(c[key], scale=60000.0)
        for key in ("fwd", "side"):
            if key in c:
                c[key + "_mm"] = [v / 360.0 * TRACKING_WHEEL_MM for v in c[key]]

        # age of each "last valid" distance value (ms since it last changed)
        self.age = {}
        for key in SENSORS:
            if key not in c:
                continue
            ages, last_change = [], 0
            vals = c[key]
            for i in range(self.n):
                if i > 0 and vals[i] != vals[i - 1]:
                    last_change = i
                ages.append(self.t[i] - self.t[last_change])
            self.age[key] = ages

    def index_at(self, t_ms):
        return max(0, min(self.n - 1, bisect.bisect_right(self.t, t_ms) - 1))

    def is_valid(self, key, v):
        return v is not None and not (key in SENSORS and v < 0)

    def implied_positions(self, i):
        """Position implied by distance sensors, computed the same way as odom_thread() in main.py.
        Returns list of (label, axis, value, heading_within_tolerance)."""
        heading = self.get("gyro", i)
        if heading is None:
            return []
        comp = compass_index(heading)
        out = []
        b1, b2 = self.get("b1", i), self.get("b2", i)
        if b1 is not None and b2 is not None and b1 >= 0 and b2 >= 0:
            d = (b1 + b2) / 2.0 - (BACK_DISTANCE1_FROM_BACK + BACK_DISTANCE2_FROM_BACK) / 2.0 + HIDDEN_PERIMETER + ROBOT_LENGTH / 2.0
            axis, val = [("X", d), ("Y", d), ("X", FIELD_MM - d), ("Y", FIELD_MM - d)][comp]
            out.append(("Back", axis, val, angle_in_tolerance(heading, 3)))
        r = self.get("r", i)
        if r is not None and r >= 0:
            m = r + HIDDEN_PERIMETER + ROBOT_WIDTH / 2.0 - RIGHT_DISTANCE_FROM_RIGHT
            axis, val = [("Y", FIELD_MM - m), ("X", m), ("Y", m), ("X", FIELD_MM - m)][comp]
            out.append(("Right", axis, val, angle_in_tolerance(heading, 1)))
        l = self.get("l", i)
        if l is not None and l >= 0:
            m = l + HIDDEN_PERIMETER + ROBOT_WIDTH / 2.0 - LEFT_DISTANCE_FROM_LEFT
            axis, val = [("Y", m), ("X", FIELD_MM - m), ("Y", FIELD_MM - m), ("X", m)][comp]
            out.append(("Left", axis, val, angle_in_tolerance(heading, 1)))
        return out


# ------------------------------------------------------------------------------ #
# Field view
# ------------------------------------------------------------------------------ #

class FieldView(tk.Canvas):
    MARGIN = 28

    def __init__(self, master, **kw):
        super().__init__(master, background="#f4f4f4", highlightthickness=0, **kw)
        self.log = None
        self.index = 0
        self.show_trail = True
        self.show_rays = True
        self._trail_px = []
        self.bind("<Configure>", lambda e: self.redraw_all())
        self.bind("<Motion>", self._on_motion)
        self.bind("<Leave>", lambda e: self.itemconfigure("hover", text=""))

    def _geometry(self):
        w, h = self.winfo_width(), self.winfo_height()
        size = max(10, min(w, h) - 2 * self.MARGIN)
        return (w - size) / 2, (h - size) / 2, size / FIELD_MM

    # NED (north, east) -> canvas pixels; North is up, East is right
    def to_px(self, n, e):
        ox, oy, s = self._geometry()
        return ox + e * s, oy + (FIELD_MM - n) * s

    def to_ned(self, px, py):
        ox, oy, s = self._geometry()
        return FIELD_MM - (py - oy) / s, (px - ox) / s

    def set_log(self, log):
        self.log = log
        self.index = 0
        self.redraw_all()

    def redraw_all(self):
        self.delete("all")
        ox, oy, s = self._geometry()
        size = FIELD_MM * s
        tiles = int(FIELD_MM // TILE_MM)
        for i in range(tiles):
            for j in range(tiles):
                x0, y0 = ox + j * TILE_MM * s, oy + i * TILE_MM * s
                self.create_rectangle(x0, y0, x0 + TILE_MM * s, y0 + TILE_MM * s,
                                      fill="#e3e3e3" if (i + j) % 2 else "#d9d9d9", outline="#bbbbbb")
        self.create_rectangle(ox, oy, ox + size, oy + size, outline="#333333", width=3)
        for k in range(tiles + 1):
            v = int(k * TILE_MM)
            px, _ = self.to_px(0, v)
            self.create_text(px, oy + size + 4, text=str(v), anchor="n", font=("Segoe UI", 7), fill="#555")
            _, py = self.to_px(v, 0)
            self.create_text(ox - 4, py, text=str(v), anchor="e", font=("Segoe UI", 7), fill="#555")
        self.create_text(ox + size / 2, oy - 4, text="North (+X)", anchor="s", font=("Segoe UI", 8, "bold"))
        self.create_text(ox + size + 4, oy + size / 2, text="E\n(+Y)", anchor="w", font=("Segoe UI", 8, "bold"))

        self._trail_px = []
        if self.log is not None and self.log.has("x") and self.log.has("y"):
            for n, e in zip(self.log.cols["x"], self.log.cols["y"]):
                self._trail_px.extend(self.to_px(n, e))
            if len(self._trail_px) >= 4:
                self.create_line(*self._trail_px, fill="#a0a0c8", width=1, dash=(2, 2), tags="fullpath")
            self.create_line(0, 0, 0, 0, fill="#3030a0", width=2, tags="trail")
        self.create_text(ox + 4, oy + 4, anchor="nw", text="", font=("Consolas", 8), fill="#222", tags="hover")
        self.update_robot(self.index)

    def _on_motion(self, ev):
        n, e = self.to_ned(ev.x, ev.y)
        if 0 <= n <= FIELD_MM and 0 <= e <= FIELD_MM:
            self.itemconfigure("hover", text="X(N) {:6.0f}  Y(E) {:6.0f}".format(n, e))
        else:
            self.itemconfigure("hover", text="")

    def update_robot(self, i):
        self.index = i
        self.delete("robot")
        log = self.log
        if log is None or not (log.has("x") and log.has("y")):
            return
        x, y = log.get("x", i), log.get("y", i)
        theta = log.get("gyro", i) or 0.0

        self.itemconfigure("fullpath", state="normal" if self.show_trail else "hidden")
        if self.show_trail and i > 0:
            self.coords("trail", *self._trail_px[:2 * (i + 1)])
            self.itemconfigure("trail", state="normal")
        else:
            self.itemconfigure("trail", state="hidden")

        if self.show_rays:
            for key, (fo, ro, ang, color, _label) in SENSORS.items():
                d = log.get(key, i)
                if d is None or d < 0:
                    continue
                sn, se = body_to_field(x, y, theta, fo, ro)
                a = math.radians(theta + ang)
                en, ee = sn + d * math.cos(a), se + d * math.sin(a)
                stale = log.age[key][i] > STALE_MS
                p0, p1 = self.to_px(sn, se), self.to_px(en, ee)
                self.create_line(*p0, *p1, fill=color, width=1 if stale else 2,
                                 dash=(4, 3) if stale else (), tags="robot")
                self.create_oval(p1[0] - 3, p1[1] - 3, p1[0] + 3, p1[1] + 3,
                                 outline=color, fill="" if stale else color, tags="robot")

        hl, hw = ROBOT_LENGTH / 2, ROBOT_WIDTH / 2
        poly = []
        for fo, ro in [(hl, -hw), (hl, hw), (-hl, hw), (-hl, -hw)]:
            poly.extend(self.to_px(*body_to_field(x, y, theta, fo, ro)))
        self.create_polygon(*poly, fill="#ffd54f", outline="#8a6d00", width=2, stipple="gray50", tags="robot")
        f1 = self.to_px(*body_to_field(x, y, theta, hl, -hw))
        f2 = self.to_px(*body_to_field(x, y, theta, hl, hw))
        self.create_line(*f1, *f2, fill="#c00000", width=4, tags="robot")
        c = self.to_px(x, y)
        tip = self.to_px(*body_to_field(x, y, theta, hl + 120, 0))
        self.create_line(*c, *tip, fill="#c00000", width=2, arrow="last", tags="robot")
        self.create_oval(c[0] - 3, c[1] - 3, c[0] + 3, c[1] + 3, fill="#000", tags="robot")
        self.tag_raise("hover")


# ------------------------------------------------------------------------------ #
# Strip chart
# ------------------------------------------------------------------------------ #

class StripChart(ttk.Frame):
    PAD_L, PAD_R, PAD_T, PAD_B = 48, 8, 16, 16

    def __init__(self, master, group, on_seek):
        super().__init__(master)
        self.on_seek = on_seek
        self.log = None
        self.window_ms = None
        self.index = 0
        self.group_var = tk.StringVar(value=group)
        top = ttk.Frame(self)
        top.pack(fill="x")
        cb = ttk.Combobox(top, textvariable=self.group_var, values=list(CHART_GROUPS), state="readonly",
                          width=22, takefocus=0)
        cb.pack(side="left")
        cb.bind("<<ComboboxSelected>>", lambda e: self.redraw())
        self.legend = ttk.Label(top, text="", font=("Consolas", 9))
        self.legend.pack(side="left", padx=6)
        self.canvas = tk.Canvas(self, background="white", highlightthickness=1, highlightbackground="#ccc", height=90)
        self.canvas.pack(fill="both", expand=True)
        self.canvas.bind("<Configure>", lambda e: self.redraw())
        self.canvas.bind("<Button-1>", self._click)
        self.canvas.bind("<B1-Motion>", self._click)
        self._yrange = {}
        self._view = (0.0, 1.0)

    def set_log(self, log):
        self.log = log
        self._yrange = {}
        self.redraw()

    def signals(self):
        if self.log is None:
            return []
        return [s for s in CHART_GROUPS[self.group_var.get()] if self.log.has(s[0])]

    def _y_range(self):
        g = self.group_var.get()
        if g not in self._yrange:
            lo, hi = math.inf, -math.inf
            for key, _, _ in self.signals():
                for v in self.log.cols[key]:
                    if self.log.is_valid(key, v):
                        lo, hi = min(lo, v), max(hi, v)
            if lo == math.inf:
                lo, hi = 0.0, 1.0
            if hi - lo < 1e-6:
                lo, hi = lo - 1, hi + 1
            pad = (hi - lo) * 0.05
            self._yrange[g] = (lo - pad, hi + pad)
        return self._yrange[g]

    def _time_window(self):
        if self.window_ms is None or self.log.duration <= self.window_ms:
            return 0.0, max(1.0, self.log.duration)
        t = self.log.t[self.index]
        t0 = min(max(0.0, t - self.window_ms / 2), self.log.duration - self.window_ms)
        return t0, t0 + self.window_ms

    def _click(self, ev):
        if self.log is None:
            return
        w = self.canvas.winfo_width()
        t0, t1 = self._view
        frac = (ev.x - self.PAD_L) / max(1, w - self.PAD_L - self.PAD_R)
        self.on_seek(t0 + min(1.0, max(0.0, frac)) * (t1 - t0))

    def set_index(self, i):
        self.index = i
        if self.log is None:
            return
        if self.window_ms is not None and self._time_window() != self._view:
            self.redraw()
        else:
            self._draw_cursor()
            self._update_legend()

    def _update_legend(self):
        parts = []
        for key, label, _ in self.signals():
            v = self.log.cols[key][self.index]
            parts.append("{}={}".format(label, "--" if not self.log.is_valid(key, v) else "{:.1f}".format(v)))
        self.legend.configure(text="  ".join(parts))

    def redraw(self):
        c = self.canvas
        c.delete("all")
        if self.log is None:
            return
        w, h = c.winfo_width(), c.winfo_height()
        pl, pr, pt, pb = self.PAD_L, self.PAD_R, self.PAD_T, self.PAD_B
        pw, ph = max(1, w - pl - pr), max(1, h - pt - pb)
        t0, t1 = self._view = self._time_window()
        y0, y1 = self._y_range()

        def tx(t):
            return pl + (t - t0) / (t1 - t0) * pw

        def ty(v):
            return pt + (1 - (v - y0) / (y1 - y0)) * ph

        for k in range(5):
            v = y0 + (y1 - y0) * k / 4
            yy = ty(v)
            c.create_line(pl, yy, pl + pw, yy, fill="#eee")
            c.create_text(pl - 3, yy, text="{:.0f}".format(v), anchor="e", font=("Segoe UI", 7), fill="#666")
        if y0 < 0 < y1:
            c.create_line(pl, ty(0), pl + pw, ty(0), fill="#bbb")
        step = self._nice_step((t1 - t0) / 8)
        tick = math.ceil(t0 / step) * step
        while tick <= t1:
            xx = tx(tick)
            c.create_line(xx, pt, xx, pt + ph, fill="#eee")
            c.create_text(xx, pt + ph + 1, text="{:g}s".format(round(tick / 1000.0, 3)), anchor="n",
                          font=("Segoe UI", 7), fill="#666")
            tick += step
        c.create_rectangle(pl, pt, pl + pw, pt + ph, outline="#999")

        # series: downsample to ~1 point per pixel, break lines at invalid (-1) values
        i0 = max(0, bisect.bisect_left(self.log.t, t0) - 1)
        i1 = min(self.log.n, bisect.bisect_right(self.log.t, t1) + 1)
        stride = max(1, (i1 - i0) // max(1, pw))
        x = pl + 4
        for key, label, color in self.signals():
            vals = self.log.cols[key]
            seg = []
            for i in range(i0, i1, stride):
                v = vals[i]
                if self.log.is_valid(key, v):
                    seg.extend((tx(self.log.t[i]), ty(v)))
                else:
                    if len(seg) >= 4:
                        c.create_line(*seg, fill=color)
                    seg = []
            if len(seg) >= 4:
                c.create_line(*seg, fill=color)
            # legend swatch
            c.create_rectangle(x, 4, x + 10, 12, fill=color, outline=color)
            c.create_text(x + 13, 8, text=label, anchor="w", font=("Segoe UI", 7))
            x += 24 + 6 * len(label)
        self._draw_cursor()
        self._update_legend()

    def _draw_cursor(self):
        c = self.canvas
        c.delete("cursor")
        if self.log is None:
            return
        t0, t1 = self._view
        w, h = c.winfo_width(), c.winfo_height()
        pw = max(1, w - self.PAD_L - self.PAD_R)
        x = self.PAD_L + (self.log.t[self.index] - t0) / (t1 - t0) * pw
        c.create_line(x, self.PAD_T, x, h - self.PAD_B, fill="#e00000", width=1, tags="cursor")

    @staticmethod
    def _nice_step(raw):
        if raw <= 0:
            return 1.0
        mag = 10 ** math.floor(math.log10(raw))
        for m in (1, 2, 5, 10):
            if raw <= m * mag:
                return m * mag
        return 10 * mag


# ------------------------------------------------------------------------------ #
# Main application
# ------------------------------------------------------------------------------ #

class App(tk.Tk):
    TICK_MS = 30

    def __init__(self, path=None):
        super().__init__()
        self.title("Aurora Log Replay")
        sw, sh = self.winfo_screenwidth(), self.winfo_screenheight()
        self.geometry("{}x{}+{}+{}".format(int(sw * 0.92), int(sh * 0.85), int(sw * 0.04), int(sh * 0.03)))
        self.minsize(900, 600)
        self.log = None
        self.index = 0
        self.rows = {}
        self.playing = False
        self.play_t = 0.0
        self._last_tick = None
        self._updating_slider = False

        self._build_menu()
        self._build_ui()
        self._bind_keys()

        if path:
            self.after(50, lambda: self.open_file(path))

    # ---- UI construction ----
    def _build_menu(self):
        m = tk.Menu(self)
        fm = tk.Menu(m, tearoff=0)
        fm.add_command(label="Open...", accelerator="Ctrl+O", command=self.open_dialog)
        fm.add_separator()
        fm.add_command(label="Exit", command=self.destroy)
        m.add_cascade(label="File", menu=fm)
        hm = tk.Menu(m, tearoff=0)
        hm.add_command(label="Keyboard shortcuts", command=self._show_help)
        m.add_cascade(label="Help", menu=hm)
        self.config(menu=m)

    def _build_ui(self):
        bar = ttk.Frame(self, padding=4)
        bar.pack(side="bottom", fill="x")
        for text, cmd in [("|<", self.go_start), ("<<", lambda: self.step(-10)), ("<", lambda: self.step(-1))]:
            ttk.Button(bar, text=text, width=4, command=cmd, takefocus=0).pack(side="left")
        self.play_btn = ttk.Button(bar, text="Play", width=7, command=self.toggle_play, takefocus=0)
        self.play_btn.pack(side="left", padx=2)
        for text, cmd in [(">", lambda: self.step(1)), (">>", lambda: self.step(10)), (">|", self.go_end)]:
            ttk.Button(bar, text=text, width=4, command=cmd, takefocus=0).pack(side="left")

        ttk.Label(bar, text="  Speed").pack(side="left")
        self.speed_var = tk.StringVar(value="1x")
        ttk.Combobox(bar, textvariable=self.speed_var, values=SPEEDS, width=6, state="readonly",
                     takefocus=0).pack(side="left", padx=2)
        ttk.Label(bar, text="  Chart window").pack(side="left")
        self.window_var = tk.StringVar(value="Full log")
        cw = ttk.Combobox(bar, textvariable=self.window_var, values=list(CHART_WINDOWS), width=8,
                          state="readonly", takefocus=0)
        cw.pack(side="left", padx=2)
        cw.bind("<<ComboboxSelected>>", lambda e: self._apply_chart_window())
        self.trail_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(bar, text="Path", variable=self.trail_var, command=self._apply_view_opts,
                        takefocus=0).pack(side="left", padx=6)
        self.rays_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(bar, text="Sensor rays", variable=self.rays_var, command=self._apply_view_opts,
                        takefocus=0).pack(side="left")
        ttk.Label(bar, text="  Go to t(s)").pack(side="left")
        self.goto_var = tk.StringVar()
        ge = ttk.Entry(bar, textvariable=self.goto_var, width=8)
        ge.pack(side="left", padx=2)
        ge.bind("<Return>", self._goto_time)
        self.time_label = ttk.Label(bar, text="No log loaded", font=("Consolas", 10))
        self.time_label.pack(side="right", padx=6)

        self.slider_var = tk.DoubleVar(value=0)
        self.slider = ttk.Scale(self, orient="horizontal", from_=0, to=1, variable=self.slider_var,
                                command=self._on_slider, takefocus=0)
        self.slider.pack(side="bottom", fill="x", padx=6)

        pw = ttk.PanedWindow(self, orient="horizontal")
        pw.pack(fill="both", expand=True)
        self.field = FieldView(pw, width=500, height=500)
        pw.add(self.field, weight=3)

        right = ttk.PanedWindow(pw, orient="vertical")
        pw.add(right, weight=4)

        tf = ttk.Frame(right)
        self.tree = ttk.Treeview(tf, columns=("value", "notes"), show="tree headings", height=14, takefocus=0)
        self.tree.heading("#0", text="Signal")
        self.tree.heading("value", text="Value")
        self.tree.heading("notes", text="Notes")
        self.tree.column("#0", width=210)
        self.tree.column("value", width=120, anchor="e")
        self.tree.column("notes", width=330)
        sb = ttk.Scrollbar(tf, orient="vertical", command=self.tree.yview)
        self.tree.configure(yscrollcommand=sb.set)
        self.tree.pack(side="left", fill="both", expand=True)
        sb.pack(side="right", fill="y")
        right.add(tf, weight=2)

        cf = ttk.Frame(right)
        self.charts = []
        for g in DEFAULT_CHARTS:
            ch = StripChart(cf, g, self.seek_time)
            ch.pack(fill="both", expand=True, pady=1)
            self.charts.append(ch)
        right.add(cf, weight=3)

    def _bind_keys(self):
        keys = {
            "<space>": self.toggle_play,
            "<Right>": lambda: self.step(1), "<Left>": lambda: self.step(-1),
            "<Shift-Right>": lambda: self.step(10), "<Shift-Left>": lambda: self.step(-10),
            "<Control-Right>": lambda: self.step(50), "<Control-Left>": lambda: self.step(-50),
            "<Home>": self.go_start, "<End>": self.go_end,
        }
        for seq, fn in keys.items():
            self.bind_all(seq, lambda e, fn=fn: self._key(fn))
        self.bind_all("<Control-o>", lambda e: self.open_dialog())

    def _key(self, fn):
        if isinstance(self.focus_get(), (ttk.Entry, tk.Entry)):
            return None
        fn()
        return "break"

    def _show_help(self):
        messagebox.showinfo("Keyboard shortcuts",
                            "Space: play / pause\n"
                            "Left / Right: step 1 sample\n"
                            "Shift+Left / Right: step 10 samples\n"
                            "Ctrl+Left / Right: step 50 samples\n"
                            "Home / End: start / end\n"
                            "Ctrl+O: open log\n\n"
                            "Click or drag on a chart to seek.\n"
                            "Dashed sensor rays = value is stale (not updated for > {} ms).".format(STALE_MS))

    # ---- file handling ----
    def open_dialog(self):
        initial = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "log")
        path = filedialog.askopenfilename(title="Open robot log",
                                          initialdir=initial if os.path.isdir(initial) else None,
                                          filetypes=[("CSV logs", "*.csv"), ("All files", "*.*")])
        if path:
            self.open_file(path)

    def open_file(self, path):
        try:
            log = LogData(path)
        except Exception as ex:  # noqa: BLE001 - report any parse problem to the user
            messagebox.showerror("Failed to open log", "{}\n\n{}".format(path, ex))
            return
        self.pause()
        self.log = log
        self.title("Aurora Log Replay - {}  ({} samples, {:.1f} s, ~{:.0f} ms/sample)".format(
            os.path.basename(path), log.n, log.duration / 1000.0, log.median_dt))
        self.slider.configure(to=max(1, log.n - 1))
        self._build_tree()
        self.field.set_log(log)
        for ch in self.charts:
            ch.set_log(log)
        self._apply_chart_window()
        self.set_index(0)

    # ---- readout ----
    def _build_tree(self):
        self.tree.delete(*self.tree.get_children())
        self.rows = {}
        log = self.log

        def group(gid, text):
            self.tree.insert("", "end", iid=gid, text=text, open=True)

        def row(gid, rid, text):
            self.tree.insert(gid, "end", iid=rid, text=text, values=("", ""))
            self.rows[rid] = True

        group("time", "Time")
        row("time", "sys_time", "sys_time (ms)")
        row("time", "t", "Log time (s)")
        row("time", "row", "Sample")
        row("time", "dt", "dt (ms)")
        group("pose", "Pose (NED)")
        row("pose", "x", "X / North (mm)")
        row("pose", "y", "Y / East (mm)")
        row("pose", "heading", "Heading (deg)")
        row("pose", "gyro", "Gyro rotation (deg)")
        row("pose", "turn_rate", "Turn rate (deg/s)")
        row("pose", "speed", "Speed (mm/s)")
        row("pose", "vn", "vN (mm/s)")
        row("pose", "ve", "vE (mm/s)")
        group("dist", "Distance sensors (last valid)")
        for key, (_, _, _, _, label) in SENSORS.items():
            if log.has(key):
                row("dist", key, "{} [{}] (mm)".format(label, key))
        group("implied", "Sensor-implied position")
        for name in ("Back", "Right", "Left"):
            row("implied", "imp_" + name, name)
        group("track", "Tracking wheels")
        for key, label in (("fwd", "Forward"), ("side", "Side")):
            if log.has(key):
                row("track", key, "{} (deg)".format(label))
        group("motors", "Drive motors")
        for key, label in MOTORS:
            if log.has(key):
                row("motors", key, "{} (turns)".format(label))
        group("other", "Other columns")
        known = {"sys_time", "x", "y", "gyro", "fwd", "side"} | set(SENSORS) | {k for k, _ in MOTORS}
        for key in log.header:
            if key not in known:
                row("other", "col_" + key, key)
        if not self.tree.get_children("other"):
            self.tree.delete("other")

    def _set_row(self, rid, value, notes=""):
        if rid in self.rows:
            self.tree.item(rid, values=(value, notes))

    def _update_tree(self, i):
        log = self.log
        g = log.get

        def fmt(v, f="{:.1f}"):
            return "--" if v is None else f.format(v)

        self._set_row("sys_time", fmt(g("sys_time", i), "{:.0f}"), "time source: " + log.time_source)
        self._set_row("t", "{:.3f}".format(log.t[i] / 1000.0), "of {:.3f}".format(log.duration / 1000.0))
        self._set_row("row", "{}".format(i + 1), "of {}".format(log.n))
        self._set_row("dt", fmt(g("dt", i) if i > 0 else None, "{:.0f}"))
        self._set_row("x", fmt(g("x", i)))
        self._set_row("y", fmt(g("y", i)))
        gyro = g("gyro", i)
        if gyro is not None:
            self._set_row("heading", "{:.2f}".format(gyro % 360.0), "compass {}".format(COMPASS[compass_index(gyro)]))
        self._set_row("gyro", fmt(gyro, "{:.2f}"))
        self._set_row("turn_rate", fmt(g("turn_rate", i)))
        self._set_row("speed", fmt(g("speed", i)))
        self._set_row("vn", fmt(g("vn", i)))
        self._set_row("ve", fmt(g("ve", i)))
        for key in SENSORS:
            v = g(key, i)
            if v is None:
                continue
            if v < 0:
                self._set_row(key, "invalid", "no valid reading yet")
            else:
                age = log.age[key][i]
                self._set_row(key, "{:.1f}".format(v),
                              "updated {:.0f} ms ago{}".format(age, "  (STALE)" if age > STALE_MS else ""))
        implied = {name: (axis, val, tol) for name, axis, val, tol in log.implied_positions(i)}
        for name in ("Back", "Right", "Left"):
            if name in implied:
                axis, val, tol = implied[name]
                odom = g(axis.lower(), i)
                diff = "odom diff {:+.1f} mm, ".format(val - odom) if odom is not None else ""
                self._set_row("imp_" + name, "{} = {:.1f}".format(axis, val),
                              "{}heading {} tolerance".format(diff, "within" if tol else "OUTSIDE"))
            else:
                self._set_row("imp_" + name, "--")
        for key in ("fwd", "side"):
            v = g(key, i)
            if v is not None:
                self._set_row(key, "{:.1f}".format(v), "{:.1f} mm travelled".format(g(key + "_mm", i)))
        for key, _ in MOTORS:
            v = g(key, i)
            if v is not None:
                rpm = g(key + "_rpm", i)
                wheel = rpm / 60.0 * DRIVE_GEAR_RATIO * DRIVE_WHEEL_MM
                self._set_row(key, "{:.3f}".format(v), "{:7.1f} rpm  (wheel {:6.0f} mm/s)".format(rpm, wheel))
        for key in log.header:
            if "col_" + key in self.rows:
                self._set_row("col_" + key, fmt(g(key, i), "{:.3f}"))

    # ---- navigation ----
    def set_index(self, i):
        if self.log is None:
            return
        i = max(0, min(self.log.n - 1, int(i)))
        self.index = i
        if not self.playing:
            self.play_t = self.log.t[i]
        self._updating_slider = True
        self.slider_var.set(i)
        self._updating_slider = False
        self.time_label.configure(text="t = {:7.3f} s   sample {:5d}/{}".format(self.log.t[i] / 1000.0, i + 1, self.log.n))
        self.field.update_robot(i)
        for ch in self.charts:
            ch.set_index(i)
        self._update_tree(i)

    def step(self, n):
        self.pause()
        self.set_index(self.index + n)

    def go_start(self):
        self.pause()
        self.set_index(0)

    def go_end(self):
        self.pause()
        if self.log:
            self.set_index(self.log.n - 1)

    def seek_time(self, t_ms):
        if self.log is None:
            return
        self.pause()
        self.set_index(self.log.index_at(t_ms))

    def _goto_time(self, _ev=None):
        try:
            self.seek_time(float(self.goto_var.get()) * 1000.0)
        except ValueError:
            self.bell()
        self.focus_set()

    def _on_slider(self, _val):
        if self._updating_slider or self.log is None:
            return
        idx = int(round(self.slider_var.get()))
        if idx != self.index:
            self.pause()
            self.set_index(idx)

    def _apply_chart_window(self):
        win = CHART_WINDOWS[self.window_var.get()]
        for ch in self.charts:
            ch.window_ms = win
            ch.redraw()

    def _apply_view_opts(self):
        self.field.show_trail = self.trail_var.get()
        self.field.show_rays = self.rays_var.get()
        self.field.update_robot(self.index)

    # ---- playback ----
    def toggle_play(self):
        if self.playing:
            self.pause()
        else:
            self.play()

    def play(self):
        if self.log is None:
            return
        if self.index >= self.log.n - 1:
            self.set_index(0)
        self.playing = True
        self.play_t = self.log.t[self.index]
        self._last_tick = time.perf_counter()
        self.play_btn.configure(text="Pause")
        self.after(self.TICK_MS, self._tick)

    def pause(self):
        self.playing = False
        self.play_btn.configure(text="Play")

    def _tick(self):
        if not self.playing or self.log is None:
            return
        now = time.perf_counter()
        speed = float(self.speed_var.get().rstrip("x"))
        self.play_t += (now - self._last_tick) * 1000.0 * speed
        self._last_tick = now
        idx = self.log.index_at(self.play_t)
        if idx != self.index:
            self.set_index(idx)
        if self.play_t >= self.log.duration:
            self.pause()
            return
        self.after(self.TICK_MS, self._tick)


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else None
    app = App(path)
    if path is None:
        app.after(100, app.open_dialog)
    app.mainloop()


if __name__ == "__main__":
    main()
