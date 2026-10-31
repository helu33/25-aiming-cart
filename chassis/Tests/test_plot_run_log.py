"""Host test for host/plot_run_log.py. Run: py -3.13 -B chassis/Tests/test_plot_run_log.py"""
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "host"))
import plot_run_log as P

HEADER = ("H,t_ms,state,mask,err_raw_dmm,err_f_dmm,v_mm_s,vmeas_mm_s,w_mrad_s,"
          "gz_mrad_s,hdg_mrad,l_tgt,l_meas,r_tgt,r_meas,duty_l,duty_r")
CRLF = chr(13) + chr(10)


def line(t_ms, state, err_dmm, mask="18", raw=None):
    raw = err_dmm if raw is None else raw
    return "D,{},{},{},{},{},200,198,0,0,0,200,199,200,201,20,21".format(
        t_ms, state, mask, raw, err_dmm)


def synthetic_run(oscillate):
    """Straight, a corner (states 2-4) at 1 s, then LINE again with a decaying wobble."""
    rows = [HEADER]
    for t in range(0, 1000, 20):
        rows.append(line(t, 1, 0))
    for t, state in ((1000, 2), (1100, 3), (1500, 4)):
        rows.append(line(t, state, 0, mask="0F", raw=-9999))
    for t in range(1600, 5000, 20):
        tau = (t - 1600) / 1000.0
        err = 120*math.exp(-tau/1.0)*math.cos(2*math.pi*1.5*tau) if oscillate else 120*math.exp(-tau/0.2)
        rows.append(line(t, 1, int(round(err))))
    rows.append(line(5000, 6, 0))
    return rows


class PlotRunLogTests(unittest.TestCase):
    def test_parse_skips_noise_and_splits_runs(self):
        lines = (["AUTO armed=1 state=0", "garbage,,", "HW imu_found=1"] + synthetic_run(True) +
                 ["AUTO LOG ON (50 Hz while running)"] + synthetic_run(False))
        runs = P.parse(lines)
        self.assertEqual(len(runs), 2)
        self.assertEqual(runs[0][0]["state"], 1)
        self.assertIsNone(runs[0][50]["err_raw"])        # -9999 means "no normal line"

    def test_corner_exit_metrics_separate_wobble_from_clean_return(self):
        wobbly = P.summarize(P.parse(synthetic_run(True))[0])
        clean = P.summarize(P.parse(synthetic_run(False))[0])
        self.assertEqual(len(wobbly["corner_exits"]), 1)
        self.assertGreaterEqual(wobbly["corner_exits"][0]["swings"], 3)
        self.assertEqual(clean["corner_exits"][0]["swings"], 0)
        self.assertAlmostEqual(wobbly["corner_exits"][0]["peak_mm"], 12.0, delta=0.5)
        self.assertLess(clean["corner_exits"][0]["settle_s"], wobbly["corner_exits"][0]["settle_s"])

    def test_serial_app_json_export_is_reassembled(self):
        # Phone app export: chunks split mid-line, written commands interleaved.
        text = CRLF.join(synthetic_run(True)) + CRLF
        chunks = [text[i:i+37] for i in range(0, len(text), 37)]
        records = [{"type": "read", "data": c, "timestamp": 1000 + i} for i, c in enumerate(chunks)]
        records.insert(3, {"type": "write", "data": "RUN", "timestamp": 1002})
        records.reverse()                                   # order must come from timestamps
        lines = P.read_lines_from_text(json.dumps(records))
        self.assertEqual(len(P.parse(lines)[0]), len(P.parse(synthetic_run(True))[0]))

    def test_plain_text_capture_still_works(self):
        lines = P.read_lines_from_text(CRLF.join(synthetic_run(False)))
        self.assertEqual(len(P.parse(lines)), 1)

    def test_plot_writes_png(self):
        run = P.parse(synthetic_run(True))[0]
        with tempfile.TemporaryDirectory() as folder:
            out = Path(folder) / "run.png"
            P.plot(run, out, "synthetic")
            self.assertGreater(out.stat().st_size, 10000)


if __name__ == "__main__":
    unittest.main(verbosity=2)
