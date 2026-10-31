"""Host test for gimbal/tools/telemetry_log.py. Run: python -B tests/test_telemetry_log.py"""
from pathlib import Path
import struct
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "gimbal" / "tools"))
import telemetry_log as T

VALID = T.FLAG_VALID
HELD = T.FLAG_SUSPENDED | T.FLAG_WAITING


def row(t, yaw=0.0, pitch=0.0, flags=VALID, rpm=0.0, gyro=0.0):
    return [t, yaw, pitch, float(flags), 0.0, 0.0, rpm, gyro, 0.0, 0.0, 0.0, 0.0, 1.5]


class ParserTest(unittest.TestCase):
    def test_frame_layout_matches_firmware(self):
        # telem_frame: time, 12 floats, then 00 00 80 7F (56 bytes)
        frame = T.pack_frame(row(1.5, 2.0))
        self.assertEqual(len(frame), 56)
        self.assertEqual(frame[-4:], b"\x00\x00\x80\x7f")
        self.assertEqual(struct.unpack("<f", frame[:4])[0], 1.5)

    def test_drops_frames_decoded_from_the_wrong_offset(self):
        bad = row(1.0)
        bad[T.COLUMNS.index("gyro_z")] = 1.8e37
        odd = row(1.0, flags=2.5)
        frames = T.JustFloatParser().feed(T.pack_frame(bad) + T.pack_frame(odd) + T.pack_frame(row(1.01, 0.3)))
        self.assertEqual(len(frames), 1)
        self.assertAlmostEqual(frames[0][1], 0.3, places=5)

    def test_resyncs_after_garbage_and_split_chunks(self):
        stream = b"\x13\x37junk" + b"".join(T.pack_frame(row(k * 0.01, k)) for k in range(5))
        p = T.JustFloatParser()
        frames = []
        for i in range(0, len(stream), 7):          # arbitrary chunking, like a serial port
            frames += p.feed(stream[i:i + 7])
        self.assertEqual([f[1] for f in frames], [0, 1, 2, 3, 4])
        self.assertGreater(p.dropped, 0)

    def test_partial_frame_before_a_tail_is_dropped(self):
        p = T.JustFloatParser()
        frames = p.feed(T.pack_frame(row(0.0))[10:] + T.pack_frame(row(0.01, 7)))
        self.assertEqual([f[1] for f in frames], [7])


class AnalyzeTest(unittest.TestCase):
    def run_with_turn(self):
        rows = [row(k * 0.01, yaw=0.2) for k in range(100)]                  # on target
        rows += [row(1.0 + k * 0.01, yaw=9.0, flags=HELD) for k in range(80)]  # turn: held
        rows += [row(1.8 + k * 0.01, yaw=3.0, flags=0) for k in range(10)]     # no frame yet
        rows += [row(1.9 + k * 0.01, yaw=2.0 - 0.05 * k) for k in range(60)]   # converging
        rows.append(row(2.5, yaw=0.1, rpm=50.0))
        return rows

    def test_metrics(self):
        m = T.analyze(self.run_with_turn())
        self.assertEqual(m["samples"], 251)
        # valid outside turns: 100 + 60 + 1 of 171
        self.assertAlmostEqual(m["valid_ratio"], 161 / 171, places=3)
        self.assertAlmostEqual(m["vis_yaw_max"], 2.0)       # held 9.0 / invalid 3.0 not counted
        self.assertEqual(len(m["turns"]), 1)
        turn = m["turns"][0]
        self.assertAlmostEqual(turn["held_s"], 0.79, places=2)
        self.assertAlmostEqual(turn["peak_after_deg"], 2.0)
        # 2.0 - 0.05k < 0.5 first at k = 31 -> 1.9 + 0.31 - 1.8
        self.assertAlmostEqual(turn["back_on_target_s"], 0.41, places=2)
        self.assertAlmostEqual(m["rpm_saturated_ratio"], 1 / 251, places=4)

    def test_turn_from_the_chassis_flag_with_vision_on(self):
        # turnhold 0: vision stays valid through the corner, only the turning flag marks it
        turning = T.FLAG_TURNING | VALID
        rows = [row(k * 0.01, yaw=0.1) for k in range(50)]
        rows += [row(0.5 + k * 0.01, yaw=0.8 if k == 10 else 0.2, flags=turning) for k in range(100)]
        rows += [row(1.5 + k * 0.01, yaw=0.1) for k in range(50)]
        m = T.analyze(rows)
        self.assertEqual(len(m["turns"]), 1)
        turn = m["turns"][0]
        self.assertAlmostEqual(turn["valid_during"], 1.0)
        self.assertAlmostEqual(turn["peak_during_deg"], 0.8)
        self.assertAlmostEqual(turn["back_on_target_s"], 0.0)
        self.assertAlmostEqual(m["valid_ratio"], 1.0)        # corners are left out

    def test_reads_files_from_before_the_motor_channels(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / "old.csv"
            path.write_text(",".join(T.COLUMNS[:T.OLD_FLOATS]) + "\n0.01,0.5,0,1,0,0,3,4,0\n")
            back = T.read_csv(path)
            self.assertEqual(back, [[0.01, 0.5, 0.0, 1.0, 0.0, 0.0, 3.0, 4.0, 0.0, 0.0, 0.0, -1.0, 0.0]])
            path.write_text(",".join(T.COLUMNS[:11]) + "\n0.01,0.5,0,1,0,0,3,4,0,2,5\n")
            self.assertEqual(T.read_csv(path)[0][9:], [2.0, 5.0, -1.0, 0.0])

    def test_csv_round_trip_and_plot(self):
        rows = self.run_with_turn()
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / "run.csv"
            T.write_csv(path, rows)
            back = T.read_csv(path)
            self.assertEqual(len(back), len(rows))
            self.assertAlmostEqual(back[-1][T.COLUMNS.index("yaw_rpm")], 50.0)
            try:
                import matplotlib  # noqa: F401
            except ImportError:
                return
            T.plot(back, Path(d) / "run.png")
            self.assertTrue((Path(d) / "run.png").stat().st_size > 1000)


class FakeLink:
    """Serial stand-in: answers command lines like the gimbal and streams frames."""

    def __init__(self):
        self.written = []
        self.pending = b""
        self.t = 0.0

    def write(self, data):
        self.written.append(data)
        line = data.decode().strip()
        if line.startswith("S "):
            _, name, value = line.split()
            self.pending += b"\n#OK " + f"{name}={float(value):g}".encode() + b"\n"
        elif line == "G":
            self.pending += b"\n#P latency=80 beta=0.3\n"

    def read(self, n):
        self.t += 0.01
        chunk = T.pack_frame(row(self.t)) + self.pending
        self.pending = b""
        return chunk

    def reset_input_buffer(self):
        pass


class CommandTest(unittest.TestCase):
    def test_reply_found_between_frames(self):
        data = T.pack_frame(row(0.0)) + b"\n#OK beta=0.3\n" + T.pack_frame(row(0.01))
        self.assertEqual(T.find_reply(data), "#OK beta=0.3")
        self.assertIsNone(T.find_reply(T.pack_frame(row(0.0)) + b"\n#OK be"))   # not complete yet
        frames = T.JustFloatParser().feed(data)
        self.assertEqual(len(frames), 2)                                        # text is skipped

    def test_frame_bytes_that_spell_a_reply_are_ignored(self):
        fake = T.pack_frame(row(0.0)) + b"#P\x93\x01\xffjunk\n" + b"x#OK \x10\n"   # binary, by chance
        self.assertIsNone(T.find_reply(fake))
        self.assertEqual(T.find_reply(fake + b"\n#P latency=80 beta=0.3\n"), "#P latency=80 beta=0.3")

    def test_set_and_get_round_trip(self):
        link = FakeLink()
        T.set_values(link, T.parse_assignments(["beta=0.3", "kpscale=2"]))
        self.assertEqual(link.written[:2], [b"S beta 0.3\n", b"S kpscale 2\n"])
        self.assertEqual(T.command(link, "G"), "#P latency=80 beta=0.3")
        with self.assertRaises(SystemExit):
            T.parse_assignments(["beta"])

    def test_record_calls_back_with_elapsed_time(self):
        link = FakeLink()
        seen = []
        rows, _ = T.record(link, 0.05, on_time=seen.append)
        self.assertTrue(rows and seen and seen == sorted(seen))

    def test_unanswered_command_is_resent(self):
        link = FakeLink()
        real_write = link.write
        lost = []

        def write(data):                    # the first line goes nowhere (link not through yet)
            if not lost:
                lost.append(data)
                link.written.append(data)
                return
            real_write(data)
        link.write = write
        self.assertEqual(T.command(link, "G", timeout=0.05), "#P latency=80 beta=0.3")
        self.assertEqual(link.written, [b"G\n", b"G\n"])

    def test_wait_for_data_stops_at_first_bytes_or_pattern(self):
        self.assertTrue(T.wait_for_data(FakeLink(), 0.5))
        silent = FakeLink()
        silent.read = lambda n: b""
        self.assertEqual(T.wait_for_data(silent, 0.05), b"")


class FakeChassis:
    """Prints its HW status line on every read; counts received bytes unless deaf."""

    def __init__(self, deaf):
        self.deaf, self.rx, self.closed = deaf, 23, False

    def read(self, n):
        return f"AUTO armed=1\r\nHW calib=1 rx_bytes={self.rx} rx_err=0\r\n".encode()

    def write(self, data):
        if not self.deaf:
            self.rx += len(data)

    def close(self):
        self.closed = True


class ChassisLinkTest(unittest.TestCase):
    def test_deaf_link_is_reopened(self):
        links = [FakeChassis(deaf=True), FakeChassis(deaf=False)]
        opened = iter(links)
        link = T.connect_chassis("COM10", 115200, open_link=lambda: next(opened), wait_s=0.05,
                                 sleep=lambda s: None)
        self.assertIs(link, links[1])
        self.assertTrue(links[0].closed)
        self.assertEqual(links[1].rx, 23 + len(b"STATUS\r\n"))

    def test_gives_up_before_any_drive(self):
        with self.assertRaises(SystemExit):
            T.connect_chassis("COM10", 115200, open_link=lambda: FakeChassis(deaf=True), wait_s=0.05,
                              sleep=lambda s: None)


class LossyChassis:
    """Loses the first `lose` command lines; keeps LAPS and answers like the firmware."""

    def __init__(self, lose):
        self.lose, self.laps, self.out, self.written = lose, 1, b"", []
        self.in_waiting = 0

    def write(self, data):
        self.written.append(data)
        if self.lose:
            self.lose -= 1
            return
        line = data.decode().strip()
        if line.startswith("LAPS "):
            self.laps = int(line[5:])
        elif line == "STATUS":
            self.out += f"AUTO armed=1 state=0 fault=0 laps={self.laps} corners=0\r\n".encode()
        elif line.startswith("TURN "):
            self.out += f"AUTO TURN turn_mm_s={int(line[5:]) * 10} rate_mrad_s=1\r\n".encode()
        elif line.startswith("SET "):
            name, value = line[4:].split()
            ok = name.lower() == "kg"
            self.out += (f"AUTO SET kg={float(value):.2f} kp=0.0200\r\n" if ok
                         else f"AUTO SET REJECTED {name.upper()} reason=name/range\r\n").encode()
        self.in_waiting = len(self.out)

    def read(self, n):
        data, self.out = self.out[:n], self.out[n:]
        self.in_waiting = len(self.out)
        return data


class ChassisSetupTest(unittest.TestCase):
    def test_lost_laps_is_resent_until_status_shows_it(self):
        link, answer = LossyChassis(lose=1), bytearray()
        T.send_chassis_checked(link, "LAPS 3", answer, wait_s=0.05)
        self.assertEqual(link.laps, 3)
        self.assertEqual(link.written.count(b"LAPS 3\r\n"), 2)

    def test_turn_is_confirmed_and_gives_up_when_never_answered(self):
        T.send_chassis_checked(LossyChassis(lose=0), "TURN 12", bytearray(), wait_s=0.05)
        with self.assertRaises(SystemExit):
            T.send_chassis_checked(LossyChassis(lose=99), "TURN 12", bytearray(), wait_s=0.05)


class ChassisSetTest(unittest.TestCase):
    def test_set_is_confirmed_and_a_rejection_stops_the_run(self):
        T.send_chassis_checked(LossyChassis(lose=1), "SET kg 1", bytearray(), wait_s=0.05)
        with self.assertRaises(SystemExit):
            T.send_chassis_checked(LossyChassis(lose=0), "SET nope 1", bytearray(), wait_s=0.05)


class OpenPortTest(unittest.TestCase):
    def test_bluetooth_timeout_is_retried(self):
        calls, sleeps = [], []

        def factory(port, baud, timeout):
            calls.append(port)
            if len(calls) < 3:
                raise OSError("could not open port 'COM13': OSError(22, 'semaphore timeout', None, 121)")
            return "link"
        self.assertEqual(T.open_port("COM13", 115200, factory=factory, sleep=sleeps.append), "link")
        self.assertEqual(len(calls), 3)
        self.assertEqual(sleeps, [T.OPEN_RETRY_S] * 2)

    def test_gives_up_with_a_hint(self):
        def factory(port, baud, timeout):
            raise OSError("could not open port 'COM13': OSError(22, 'semaphore timeout', None, 121)")
        with self.assertRaises(SystemExit) as e:
            T.open_port("COM13", 115200, factory=factory, sleep=lambda s: None)
        self.assertIn("Bluetooth off/on", str(e.exception))

    def test_missing_port_is_not_retried(self):
        calls = []

        def factory(port, baud, timeout):
            calls.append(port)
            raise OSError("could not open port 'COM9': FileNotFoundError(2, 'not found', None, 2)")
        with self.assertRaises(SystemExit) as e:
            T.open_port("COM9", 115200, factory=factory, sleep=lambda s: None)
        self.assertEqual(calls, ["COM9"])
        self.assertIn("does not exist", str(e.exception))


if __name__ == "__main__":
    unittest.main(verbosity=1)
