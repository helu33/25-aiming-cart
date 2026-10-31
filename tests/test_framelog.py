"""Host test for vision/framelog.py and vision/tools/framelog_report.py.
Run: python -B tests/test_framelog.py"""
from pathlib import Path
import io
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "vision"))
sys.path.insert(0, str(ROOT / "vision" / "tools"))
sys.path.insert(0, str(ROOT / "gimbal" / "tools"))
import framelog as F
import framelog_report as FR
import telemetry_log as T

COL = {name: i for i, name in enumerate(T.COLUMNS)}


class Detector:
    last_status, last_candidates, threshold, last_area_fraction = "confirmed", 2, 70.0, 0.0123


class MemoryFile(io.StringIO):
    def __init__(self, store):
        super().__init__()
        self.store = store

    def close(self):
        self.store.append(self.getvalue())
        super().close()


class FrameLogTest(unittest.TestCase):
    def test_line_format_and_comma_free_reason(self):
        line = F.frame_line(1234, 31, {"valid": False, "reason": "pose reprojection error 4.1>3.0px, x"},
                            Detector())
        fields = line.split(",")
        self.assertEqual(len(fields), len(F.HEADER.split(",")))
        self.assertEqual(fields[:4], ["1234", "31", "0", "pose reprojection error 4.1>3.0px; x"])
        self.assertEqual(fields[6:8], ["70", "1.23"])
        self.assertEqual(fields[8:], ["", "", ""])
        ok = F.frame_line(1, 2, {"valid": True, "tilt_deg": 12.34, "reprojection_px": 0.5, "depth_mm": 812.4},
                          Detector()).split(",")
        self.assertEqual(ok[2], "1")
        self.assertEqual(ok[8:], ["12.3", "0.50", "812"])

    def test_buffered_until_due_then_appended_once_with_header(self):
        written = []
        log = F.FrameLog("x.csv", flush_ms=1000, opener=lambda path, mode: MemoryFile(written))
        for t in range(0, 900, 40):
            log.add(t, 30, {"valid": True}, Detector())
        self.assertFalse(log.due(899))
        self.assertTrue(log.due(1000))
        self.assertEqual(log.flush(1000), 1 + 23)
        log.add(1040, 30, {"valid": False, "reason": "no_target"}, Detector())
        log.flush(2000)
        text = "".join(written)
        self.assertEqual(text.count(F.HEADER), 1)
        self.assertEqual(len(text.strip().splitlines()), 1 + 24)
        self.assertEqual(log.flush(), 0)            # nothing pending


class ReportTest(unittest.TestCase):
    def frames_and_gimbal(self, offset):
        frames, gimbal = [], []
        def valid_at(t):                            # dropouts of different lengths
            return not any(a <= t < b for a, b in ((2.0, 2.3), (5.0, 5.12), (7.5, 8.2)))
        t = 0.0
        while t < 10.0:
            frames.append({"t": t, "valid": valid_at(t), "reason": "", "det": "confirmed", "area_pct": 1.0})
            t += 0.04
        for k in range(1000):
            g = [0.0] * len(T.COLUMNS)
            g[0] = offset + k * 0.01
            g[COL["flags"]] = float(T.FLAG_VALID if valid_at(k * 0.01 - 0.005) else 0)
            gimbal.append(g)
        return frames, gimbal

    def test_aligns_the_k230_clock_on_the_valid_flags(self):
        frames, gimbal = self.frames_and_gimbal(offset=123.45)
        offset, agreement = FR.align_offset(frames, gimbal, COL)
        self.assertAlmostEqual(offset, 123.45, delta=0.03)
        self.assertGreater(agreement, 0.95)

    def test_outcome_and_sections(self):
        self.assertEqual(FR.outcome({"valid": False, "det": "no_candidates", "reason": "no_target"}), "no_candidates")
        self.assertEqual(FR.outcome({"valid": False, "det": "confirmed", "reason": "pose reprojection error 5>3px"}),
                         "pose_reprojection")
        self.assertEqual(FR.outcome({"valid": True}), "valid")
        spans = [(10.0, 11.5), (14.0, 15.5)]
        self.assertEqual(FR.section_of(9.0, spans), "start edge")
        self.assertEqual(FR.section_of(10.5, spans), "corner 1")
        self.assertEqual(FR.section_of(12.0, spans), "edge after corner 1, 0-1 s")
        self.assertEqual(FR.section_of(13.9, spans), "edge after corner 1, 2+ s")
        self.assertEqual(FR.section_of(15.0, spans), "corner 2")

    def test_reads_a_log_with_a_repeated_header(self):
        text = F.HEADER + "\n1000,30,1,tracking,confirmed,1,70,1.50,10.0,0.40,900\n" + \
               F.HEADER + "\n2000,31,0,no_target,no_candidates,0,55,0.00,,,\n"
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / "f.csv"
            path.write_text(text)
            frames = FR.read_framelog(path)
        self.assertEqual([f["t"] for f in frames], [1.0, 2.0])
        self.assertEqual(frames[1]["tilt_deg"], None)
        self.assertIn("valid  50.0 %", FR.summarise(frames)[0])


if __name__ == "__main__":
    unittest.main()
