"""Per-frame outcome log for diagnosing vision dropouts on the track (K230, off by default).

The once-a-second status line only shows the last frame; the dropouts seen while driving
(2026-10-28: 59-75 % valid frames in corner 2 and on the far edge) last a few frames, so
this records every frame: one CSV line kept in RAM and appended to a file on the SD card
every flush_ms (the write blocks the loop for a few ms, hence off unless config.FRAME_LOG).
Pure Python without f-strings so the same file runs under MicroPython and the PC tests.

Columns (HEADER): t_ms board ticks; ms acquisition+processing; valid sent to the gimbal;
reason (processor/runtime reason, commas replaced); det detector status; cand rectangles
found before association with the previous one; thr binarisation threshold; area_pct the
chosen (or largest) outer quad as % of the image; tilt_deg, reproj_px, depth_mm of the pose
(empty when no pose was computed).
"""
HEADER = "t_ms,ms,valid,reason,det,cand,thr,area_pct,tilt_deg,reproj_px,depth_mm"


def _number(value, digits):
    if value is None:
        return ""
    try:
        return ("{:." + str(digits) + "f}").format(float(value))
    except (TypeError, ValueError):
        return ""


def frame_line(t_ms, elapsed_ms, result, detector):
    reason = str(result.get("reason", "")).replace(",", ";")
    return ",".join((
        str(int(t_ms)), str(int(elapsed_ms)), "1" if result.get("valid") else "0", reason,
        str(getattr(detector, "last_status", "")), str(getattr(detector, "last_candidates", "")),
        _number(getattr(detector, "threshold", None), 0),
        _number(100.0 * getattr(detector, "last_area_fraction", 0.0), 2),
        _number(result.get("tilt_deg"), 1), _number(result.get("reprojection_px"), 2),
        _number(result.get("depth_mm"), 0)))


class FrameLog:
    def __init__(self, path, flush_ms=10000, opener=open):
        self.path, self.flush_ms, self.opener = path, flush_ms, opener
        self.pending = [HEADER]
        self.last_flush = None
        self.lines_written = 0

    def add(self, t_ms, elapsed_ms, result, detector):
        self.pending.append(frame_line(t_ms, elapsed_ms, result, detector))
        if self.last_flush is None:
            self.last_flush = t_ms

    def due(self, t_ms):
        return self.last_flush is not None and t_ms - self.last_flush >= self.flush_ms

    def flush(self, t_ms=None):
        if not self.pending:
            return 0
        with self.opener(self.path, "a") as f:
            f.write("\n".join(self.pending) + "\n")
        written = len(self.pending)
        self.lines_written += written
        self.pending = []
        if t_ms is not None:
            self.last_flush = t_ms
        return written
