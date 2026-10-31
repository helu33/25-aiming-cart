"""Host test for gimbal/tools/run_report.py. Run: python -B tests/test_run_report.py"""
from pathlib import Path
import math
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "gimbal" / "tools"))
import run_report as R
import telemetry_log as T

COL = {name: i for i, name in enumerate(T.COLUMNS)}


def chassis_row(t_ms, state, gz_deg_s=0.0, err_mm=0.0):
    return {"t_ms": t_ms, "state": state, "err_f": int(err_mm * 10), "gz": int(gz_deg_s / R.MRAD_TO_DEG),
            "v": 350, "vmeas": 350}


def gimbal_row(t, flags=T.FLAG_VALID | T.FLAG_TRACKING, yaw=0.0, cmd=0.0, motor=0.0, gz=0.0):
    r = [0.0] * len(T.COLUMNS)
    r[0], r[COL["flags"]], r[COL["vis_yaw"]] = t, float(flags), yaw
    r[COL["yaw_rpm"]], r[COL["motor_rpm"]], r[COL["chassis_gz"]] = cmd, motor, gz
    return r


class ChassisLogTest(unittest.TestCase):
    def test_reads_d_lines_and_skips_the_rest(self):
        text = ("AUTO LOG ON\r\nH,t_ms,state\r\nD," + ",".join(["1"] * 16) + "\r\n"
                "D,2,1,3\r\nnoise\r\nD," + ",".join(["5"] * 16) + "\r\n")
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / "c.txt"
            path.write_text(text)
            rows = R.read_chassis_log(path)
        self.assertEqual([r["t_ms"] for r in rows], [1, 5])

    def test_mask_is_hex(self):
        # The chassis prints the probe mask as %02X: "1F" (all probes, e.g. PRE_TURN) is a row.
        line = "D,2340,2,1F,-9999,0,176,204,0,-11,-23,0,0,0,0,0,0\r\n"
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / "c.txt"
            path.write_text(line + line.replace(",1F,", ",10,"))
            rows = R.read_chassis_log(path)
        self.assertEqual([(r["state"], r["mask"]) for r in rows], [(R.PRE_TURN, 0x1F), (R.PRE_TURN, 0x10)])

    def test_kicks_and_exit_error(self):
        rows = [chassis_row(t, R.LINE) for t in range(0, 2000, 20)]               # settle + calm
        rows += [chassis_row(2000 + k * 20, R.LINE, gz_deg_s=20.0 if k == 5 else 2.0) for k in range(50)]
        rows += [chassis_row(3000 + k * 20, R.TURN, gz_deg_s=100.0) for k in range(30)]
        rows += [chassis_row(3600 + k * 20, R.REACQUIRE, err_mm=15.0) for k in range(10)]
        rows += [chassis_row(3800 + k * 20, R.LINE, err_mm=5.0) for k in range(100)]
        m = R.chassis_metrics(rows)
        self.assertEqual(m["corners (chassis)"], 1.0)
        self.assertAlmostEqual(m["exit line error max, mm"], 15.0)
        self.assertAlmostEqual(m["body yaw max on straights, deg/s"], 20.0, places=0)
        self.assertGreater(m["kicks > 12 deg/s per minute"], 0.0)
        self.assertEqual(R.chassis_corners(rows), [(3.0, 3.78)])


class GimbalTest(unittest.TestCase):
    def test_corner_spans_merge_blips_and_drop_short_ones(self):
        turning = T.FLAG_VALID | T.FLAG_TURNING
        rows = [gimbal_row(k * 0.01) for k in range(100)]
        rows += [gimbal_row(1.0 + k * 0.01, turning) for k in range(60)]
        rows += [gimbal_row(1.6 + k * 0.01) for k in range(20)]                   # 0.2 s blip: merged
        rows += [gimbal_row(1.8 + k * 0.01, turning) for k in range(60)]
        rows += [gimbal_row(2.4 + k * 0.01) for k in range(200)]
        rows += [gimbal_row(4.4 + k * 0.01, turning) for k in range(10)]          # 0.1 s: dropped
        rows += [gimbal_row(4.5 + k * 0.01) for k in range(50)]
        spans = R.corner_spans(rows, COL)
        self.assertEqual(len(spans), 1)
        self.assertAlmostEqual(spans[0][0], 1.0)
        self.assertAlmostEqual(spans[0][1], 2.39, places=2)

    def test_motor_lag_fit_recovers_a_first_order_motor(self):
        rows, motor = [], 0.0
        for k in range(600):
            t = k * 0.01
            cmd = 20.0 if (k // 100) % 2 else -10.0
            rows.append(gimbal_row(t, cmd=cmd, motor=motor))
            motor += (0.8 * cmd - motor) * 0.01 / 0.13
        tau, gain, residual = R.fit_motor_lag(rows, COL)
        self.assertAlmostEqual(tau, 0.13)
        self.assertAlmostEqual(gain, 0.8)
        self.assertLess(residual, 0.2)

    def test_chassis_clock_offset_from_the_gz_channel(self):
        def body(t):                       # chassis gz: two corners
            return 100.0 if 2.0 <= t % 6.0 < 3.0 else 0.0
        offset = 40.3
        chassis = [chassis_row(int(t * 1000), R.TURN if body(t) else R.LINE, gz_deg_s=body(t))
                   for t in (k * 0.02 for k in range(600))]
        turning = T.FLAG_VALID | T.FLAG_TURNING
        rows = [gimbal_row(offset + t, turning if body(t) else T.FLAG_VALID, gz=body(t))
                for t in (k * 0.01 for k in range(1200))]
        self.assertAlmostEqual(R.chassis_offset(rows, COL, chassis), offset, places=1)

    def test_vision_sections_from_the_code_channel(self):
        turning = T.FLAG_VALID | T.FLAG_TURNING
        rows = []                                    # 0-3 s straight, 3-4.5 s corner 1, then straight
        for k in range(800):
            t = k * 0.01
            in_corner = 3.0 <= t < 4.5
            code = 1 if in_corner and k % 2 else 0   # half of the corner: no rectangle found
            r = gimbal_row(t, turning if in_corner else T.FLAG_VALID)
            r[COL["vis_code"]], r[COL["vis_area"]] = float(code), 0.0 if code else 1.25
            rows.append(r)
        with tempfile.TemporaryDirectory() as d:
            stem = str(Path(d) / "v")
            T.write_csv(stem + ".csv", rows)
            lines = []
            R.vision_sections(stem, lines.append)
        corner = next(line for line in lines if line.startswith("corner 1"))
        self.assertIn(" 50.0%", corner)
        self.assertIn("no rectangle 50%", corner)
        self.assertIn("1.25/ 0.00", corner)
        edge = next(line for line in lines if line.startswith("edge after corner 1, 0-1 s"))
        self.assertIn("100.0%", edge)

    def test_offset_falls_back_to_turn_flags_when_channel_10_is_not_gz(self):
        def body(t):
            return 100.0 if 2.0 <= t % 6.0 < 3.0 else 0.0
        offset = 40.3
        chassis = [chassis_row(int(t * 1000), R.TURN if body(t) else R.LINE, gz_deg_s=body(t))
                   for t in (k * 0.02 for k in range(600))]
        turning = T.FLAG_VALID | T.FLAG_TURNING
        rows = [gimbal_row(offset + t, turning if body(t) else T.FLAG_VALID, gz=5.0 * math.sin(7 * t))
                for t in (k * 0.01 for k in range(1200))]           # channel 10: some other signal
        self.assertAlmostEqual(R.chassis_offset(rows, COL, chassis), offset, places=1)
        self.assertAlmostEqual(R.chassis_offset(rows, COL, chassis, use_gz=False), offset, places=1)

    def test_turn_flag_offset_starts_at_pre_turn(self):
        # Flag 16 is the chassis "hold vision": PRE_TURN, TURN and REACQUIRE, so it rises at
        # PRE_TURN (0.18 s before TURN here), not at TURN.
        def state(t):
            phase = t % 6.0
            if 1.82 <= phase < 2.0:
                return R.PRE_TURN
            return R.TURN if 2.0 <= phase < 3.0 else R.LINE
        offset = 40.3
        chassis = [chassis_row(int(t * 1000), state(t)) for t in (k * 0.02 for k in range(600))]
        turning = T.FLAG_VALID | T.FLAG_TURNING
        rows = [gimbal_row(offset + t, turning if state(t) != R.LINE else T.FLAG_VALID)
                for t in (k * 0.01 for k in range(1200))]
        self.assertAlmostEqual(R.chassis_offset(rows, COL, chassis, use_gz=False), offset, places=1)

    def test_report_columns(self):
        text = R.format_report(["a", "b"], [{"x": 1.0}, {"x": 2.0, "y": 3.0}])
        self.assertIn("a", text.splitlines()[0])
        self.assertTrue(text.splitlines()[2].strip().startswith("y"))
        self.assertIn("nan", text.splitlines()[2])


if __name__ == "__main__":
    unittest.main()
