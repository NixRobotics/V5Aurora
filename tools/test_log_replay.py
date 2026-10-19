import csv
import math
from pathlib import Path
import tempfile
import tkinter as tk
from types import SimpleNamespace
import unittest

from log_replay import (FieldView, LogData, STATUS_LABELS, STATUS_SHIFTS, TRACKING_WHEEL_MM,
                        TRACKING_FORWARD_OFFSET, TRACKING_SIDE_OFFSET,
                        DRIVE_GEAR_RATIO, DRIVE_WHEEL_MM, DRIVE_FORWARD_EFFICIENCY,
                        integrate_body_step)


class DistanceStatusTests(unittest.TestCase):
    def load(self, header, rows):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "log.csv"
            with path.open("w", newline="") as stream:
                writer = csv.writer(stream)
                writer.writerow(header)
                writer.writerows(rows)
            return LogData(path)

    def test_nibble_order_and_all_codes(self):
        for key, shift in STATUS_SHIFTS.items():
            for code in range(16):
                with self.subTest(sensor=key, code=code):
                    log = self.load(["sys_time", key, "status"], [[100, 123, code << shift]])
                    self.assertEqual(log.sensor_status(key, 0), code)
                    self.assertEqual(log.sensor_dashed(key, 0), code not in (0, 5))
                    self.assertEqual(log.get(key + "_status", 0), 0 if code == 5 else code)
                    self.assertIn(STATUS_LABELS.get(code, "Unknown code {}".format(code)),
                                  log.sensor_notes(key, 0))

    def test_not_updated_is_neutral_even_for_unchanged_readings(self):
        for period in (10, 20):
            with self.subTest(period=period):
                log = self.load(["sys_time", "r", "status"],
                                [[i * period, 100, 5] for i in range(30)])
                self.assertGreater(log.age["r"][-1], 150)
                self.assertFalse(log.sensor_dashed("r", 29))
                self.assertNotIn("STALE", log.sensor_notes("r", 29))
                self.assertEqual(log.index_at(15 * period), 15)

    def test_fault_then_not_updated_does_not_carry_fault_forward(self):
        log = self.load(["sys_time", "r", "status"], [[0, 100, 1], [20, 101, 5], [40, 101, 0]])
        self.assertTrue(log.sensor_dashed("r", 0))
        self.assertFalse(log.sensor_dashed("r", 1))
        self.assertEqual(log.get("r", 1), 101)

    def test_legacy_age_behavior(self):
        log = self.load(["sys_time", "r"], [[0, 100], [200, 100]])
        self.assertIsNone(log.sensor_status("r", 1))
        self.assertTrue(log.sensor_dashed("r", 1))
        self.assertIn("unchanged", log.sensor_notes("r", 1))

    def test_invalid_status_is_reported(self):
        for value in (-1, 65536, 1.5, "nan", "inf"):
            with self.subTest(value=value), self.assertRaisesRegex(ValueError, "Invalid packed distance status"):
                self.load(["sys_time", "r", "status"], [[0, 100, value]])

    def test_sample_log(self):
        log = LogData(Path(__file__).resolve().parents[1] / "log" / "log_new_format.csv")
        self.assertEqual([log.sensor_status(key, 0) for key in STATUS_SHIFTS], [5, 5, 2, 5])
        self.assertEqual(log.get("l", 0), -1)
        self.assertIn("No object", log.sensor_notes("l", 0))
        self.assertEqual(log.get("status", 0), 21797)

    def test_straight_motion_at_different_sample_rates(self):
        for period in (10, 20, 27):
            with self.subTest(period=period):
                rows = []
                motor_scale = DRIVE_GEAR_RATIO * DRIVE_WHEEL_MM * math.sqrt(2) * DRIVE_FORWARD_EFFICIENCY
                for i in range(40):
                    distance = i * period * 0.5
                    turns = distance / motor_scale
                    rows.append([1000 + i * period, 10, 20, 0, distance / TRACKING_WHEEL_MM * 360, 0,
                                 turns, turns, turns, turns])
                log = self.load(["sys_time", "x", "y", "gyro_value", "fwd_value", "side_value",
                                 "lfm_value", "lbm_value", "rfm_value", "rbm_value"], rows)
                self.assertAlmostEqual(log.get("track_vf", 30), 500)
                self.assertAlmostEqual(log.get("motor_vf", 30), 500)
                self.assertAlmostEqual(log.get("residual_vf", 30), 0)
                self.assertAlmostEqual(log.get("trajectory_gap", 39), 0)
                self.assertAlmostEqual(log.trajectories["track"][-1][0], 10 + 39 * period * 0.5)
                self.assertIsNone(log.get("track_vf", 0))

    def test_pure_rotation_removes_tracking_offset_travel(self):
        rows = []
        for i in range(101):
            angle = i * math.pi / 50
            fwd = -TRACKING_FORWARD_OFFSET * angle / TRACKING_WHEEL_MM * 360
            side = TRACKING_SIDE_OFFSET * angle / TRACKING_WHEEL_MM * 360
            rows.append([i * 10, 500, 600, math.degrees(angle), fwd, side, i, i, -i, -i])
        log = self.load(["sys_time", "x", "y", "gyro", "fwd", "side", "lfm", "lbm", "rfm", "rbm"], rows)
        for name in ("track", "motor"):
            for n, e in log.trajectories[name]:
                self.assertAlmostEqual(n, 500)
                self.assertAlmostEqual(e, 600)
        self.assertAlmostEqual(log.get("track_vf", 50), 0)
        self.assertAlmostEqual(log.get("track_vr", 50), 0)

    def test_arc_integration_and_ned_direction(self):
        n, e = integrate_body_step(0, 0, 0, 90, 100 * math.pi / 2, 0)
        self.assertAlmostEqual(n, 100)
        self.assertAlmostEqual(e, 100)
        self.assertEqual(integrate_body_step(0, 0, 0, 0, 0, 100), (0, 100))
        n, e = integrate_body_step(0, 0, 0, -90, 100 * math.pi / 2, 0)
        self.assertAlmostEqual(n, 100)
        self.assertAlmostEqual(e, -100)

    def test_strafe_mixing(self):
        motor_scale = DRIVE_GEAR_RATIO * DRIVE_WHEEL_MM * math.sqrt(2)
        turns = 100 / motor_scale
        log = self.load(["sys_time", "x", "y", "gyro", "fwd", "side", "lfm", "lbm", "rfm", "rbm"],
                        [[0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                         [200, 0, 0, 0, 0, 100 / TRACKING_WHEEL_MM * 360, turns, -turns, -turns, turns]])
        self.assertAlmostEqual(log.get("motor_vr", 1), 500)
        self.assertAlmostEqual(log.get("track_vr", 1), 500)
        self.assertAlmostEqual(log.get("trajectory_gap", 1), 0)

    def test_distance_intervals_are_sparse_observed_changes(self):
        log = self.load(["sys_time", "r", "status"],
                        [[0, 100, 0], [20, 101, 5], [40, 101, 5], [60, 102, 0],
                         [80, 102, 1], [100, 103, 5], [120, 104, 0]])
        self.assertEqual(log.cols["r_change_ms"], [None, None, None, 40, None, None, 20])

    def test_partial_logs_do_not_invent_motion(self):
        log = self.load(["sys_time", "r"], [[0, 100], [20, 101]])
        self.assertEqual(log.trajectories, {})
        self.assertFalse(log.has("residual_vf"))

    def test_reconstruction_is_not_corrected_by_logged_pose(self):
        scale = DRIVE_GEAR_RATIO * DRIVE_WHEEL_MM * math.sqrt(2) * DRIVE_FORWARD_EFFICIENCY
        turns = 100 / scale
        log = self.load(["sys_time", "x", "y", "gyro", "fwd", "side", "lfm", "lbm", "rfm", "rbm"],
                        [[0, 10, 20, 0, 0, 0, 0, 0, 0, 0],
                         [200, 900, 800, 0, 90 / TRACKING_WHEEL_MM * 360, 0,
                          turns, turns, turns, turns]])
        self.assertAlmostEqual(log.trajectories["track"][-1][0], 100)
        self.assertAlmostEqual(log.trajectories["motor"][-1][0], 110)
        self.assertAlmostEqual(log.get("trajectory_gap", 1), 10)
        self.assertAlmostEqual(log.get("residual_vf", 1), -50)


class FieldZoomTests(unittest.TestCase):
    def setUp(self):
        self.root = tk.Tk()
        self.addCleanup(self.root.destroy)
        self.root.geometry("700x600")
        self.field = FieldView(self.root)
        self.field.pack(fill="both", expand=True)
        self.root.update()

    def assertPointAlmostEqual(self, first, second):
        for a, b in zip(first, second):
            self.assertAlmostEqual(a, b)

    def test_zoom_anchors_pointer_and_limits(self):
        field = self.field
        point = field.to_ned(230, 410)
        field.zoom_at(2, 230, 410)
        self.assertEqual(field.zoom, 2)
        self.assertPointAlmostEqual(field.to_px(*point), (230, 410))
        field.zoom_at(100, 230, 410)
        self.assertEqual(field.zoom, field.MAX_ZOOM)
        self.assertPointAlmostEqual(field.to_px(*point), (230, 410))
        field.zoom_at(0.001, 230, 410)
        self.assertEqual(field.zoom, 1)
        self.assertPointAlmostEqual(field.to_px(*point), (230, 410))

    def test_pan_reset_and_resize(self):
        field = self.field
        field.zoom_at(3)
        before = field.to_px(1200, 2300)
        field._start_pan(SimpleNamespace(x=100, y=100))
        field._pan(SimpleNamespace(x=150, y=70))
        field._end_pan(None)
        self.assertPointAlmostEqual(field.to_px(1200, 2300), (before[0] + 50, before[1] - 30))
        center = (field.center_n, field.center_e)
        self.root.geometry("800x500")
        self.root.update()
        self.assertEqual(field.zoom, 3)
        self.assertEqual(center, (field.center_n, field.center_e))
        self.assertPointAlmostEqual(field.to_px(*center),
                                    (field.winfo_width() / 2, field.winfo_height() / 2))
        field.reset_view()
        self.assertEqual(field.zoom, 1)
        self.assertEqual((field.center_n, field.center_e), (1800, 1800))

    def test_paths_and_playback_use_zoom_transform(self):
        field = self.field
        base = Path(__file__).resolve().parents[1] / "log"
        log = LogData(base / "log_new_format.csv")
        field.set_log(log)
        field.show_diagnostics = True
        field.zoom_at(4, 200, 200)
        field.update_robot(100)
        self.assertPointAlmostEqual(field.coords("trail")[-2:],
                                    field.to_px(log.get("x", 100), log.get("y", 100)))
        for name, path in log.trajectories.items():
            self.assertPointAlmostEqual(field.coords("path_" + name)[-2:], field.to_px(*path[100]))
        self.assertEqual(field.zoom, 4)
        field.set_log(LogData(base / "ai_310mm.csv"))
        field.update_robot(10)
        self.assertEqual(field._diagnostic_px, {})


if __name__ == "__main__":
    unittest.main()
