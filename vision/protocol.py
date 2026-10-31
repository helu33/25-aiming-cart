"""MicroPython-compatible implementation of the production $V protocol."""

def finite(value):
    return value == value and -1.0e30 < value < 1.0e30


def crc16(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ (0x1021 if crc & 0x8000 else 0)) & 0xFFFF
    return crc


# Why a frame is not valid ($V version 3, telemetry channel vis_code on the gimbal), in
# pipeline order. Field 2026-10-28: 59-75 % valid frames in corner 2 / on the far edge.
CODE_VALID = 0
CODE_NO_RECTANGLE = 1        # detector found no tape rectangle (det no_candidates)
CODE_ASSOCIATION = 2         # rectangles found, none near the tracked one
CODE_CONFIRMING = 3          # new track, waiting for confirm_frames
CODE_REPROJECTION = 4        # pose reprojection error over the gate
CODE_TILT = 5
CODE_DEPTH = 6
CODE_ANGLE = 7
CODE_TOO_OLD = 8             # acquisition + processing slower than MAX_RESULT_AGE_MS
CODE_OTHER = 9               # geometry / calibration / capture mode
_REASON_CODES = (("pose reprojection", CODE_REPROJECTION), ("target_tilt", CODE_TILT),
                 ("target_depth", CODE_DEPTH), ("angular_error", CODE_ANGLE),
                 ("frame_too_old", CODE_TOO_OLD), ("confirming", CODE_CONFIRMING))


def failure_code(result):
    """CODE_* for a processor/runtime result (uses its "det" detector status when present)."""
    if result.get("valid"):
        return CODE_VALID
    reason = str(result.get("reason", ""))
    for prefix, code in _REASON_CODES:
        if reason.startswith(prefix):
            return code
    det = result.get("det")
    if det == "association_rejected":
        return CODE_ASSOCIATION
    if det == "confirming":
        return CODE_CONFIRMING
    if reason == "no_target":
        return CODE_NO_RECTANGLE
    return CODE_OTHER


def encode_vision(seq, valid, yaw=0.0, pitch=0.0, range_mm=0.0, age_ms=0, code=None, area_pct=None):
    """$V version 2: ...,yaw,pitch,range_mm,age_ms. range_mm: camera to target point (0 when
    no target), age_ms: snapshot to send on the K230; both feed the gimbal's latency
    compensation and vehicle-speed feed-forward. With code (CODE_*) and area_pct (outer
    rectangle / image, %): version 3, ...,age_ms,code,area_x100, for the gimbal telemetry."""
    if type(seq) is not int or not 0 <= seq <= 65535:
        raise ValueError("invalid sequence")
    if type(valid) is not bool:
        raise ValueError("valid must be bool")
    if not valid:
        yaw = pitch = range_mm = 0.0
    if not finite(yaw) or not finite(pitch) or abs(yaw) > 180 or abs(pitch) > 180:
        raise ValueError("invalid angle")
    if not finite(range_mm) or not finite(age_ms):
        raise ValueError("invalid range/age")
    range_mm = min(20000, max(0, int(range_mm + 0.5)))
    age_ms = min(1000, max(0, int(age_ms + 0.5)))
    if code is None:
        body = "V,2,{:d},{:d},{:.3f},{:.3f},{:d},{:d}".format(
            seq, int(valid), yaw, pitch, range_mm, age_ms).encode("ascii")
    else:
        if type(code) is not int or not 0 <= code <= 99:
            raise ValueError("invalid code")
        area = area_pct if area_pct is not None and finite(area_pct) else 0.0
        area_x100 = min(10000, max(0, int(area * 100.0 + 0.5)))
        body = "V,3,{:d},{:d},{:.3f},{:.3f},{:d},{:d},{:d},{:d}".format(
            seq, int(valid), yaw, pitch, range_mm, age_ms, code, area_x100).encode("ascii")
    return b"$" + body + "*{:04X}\n".format(crc16(body)).encode("ascii")


class VisionSender:
    def __init__(self, uart):
        self.uart = uart
        self.seq = 0
        self.failures = 0

    def send(self, valid, yaw=0.0, pitch=0.0, range_mm=0.0, age_ms=0, code=None, area_pct=None):
        frame = encode_vision(self.seq, valid, yaw, pitch, range_mm, age_ms, code, area_pct)
        self.seq = (self.seq + 1) & 0xFFFF
        try:
            count = self.uart.write(frame)
            if count != len(frame):
                # Terminate an incomplete line so the receiver can resynchronize.
                self.uart.write(b"\n")
                self.failures += 1
                return False
        except OSError:
            self.failures += 1
            return False
        return True


def decode_task(line):
    """$K,1,<seq>,<task>,<progress>*CRC16 from the gimbal (line without CR/LF, bytes): the chassis
    task (0 bullseye, 1 circle) and run progress in laps, or None for a malformed frame."""
    try:
        text = line.decode() if isinstance(line, bytes) else line
    except UnicodeError:
        return None
    star = text.find("*")
    if not text.startswith("$K,") or star < 0 or len(text) - star - 1 != 4:
        return None
    try:
        received = int(text[star + 1:], 16)
    except ValueError:
        return None
    if crc16(text[1:star].encode()) != received:
        return None
    fields = text[1:star].split(",")
    if len(fields) != 5 or fields[1] != "1":
        return None
    try:
        seq, task, progress = int(fields[2]), int(fields[3]), int(fields[4])
    except ValueError:
        return None
    if not (0 <= seq <= 65535 and task in (0, 1) and 0 <= progress <= 9999):
        return None
    return task, progress / 1000.0


class TaskReceiver:
    """$K frames from the gimbal on the same UART, read once per camera frame. It must never wait:
    on the board the first version (plain read()) took the K230 from ~23 to ~12 fps. So read only
    the bytes any() reports (UART also created with timeout 0 where the firmware takes it). Lines
    are cut at LF, resynchronised at the last '$'; poll() returns the newest (task, progress)."""
    MAX_PENDING = 48
    MAX_READ = 512                       # bytes per poll (~1 s of $K): bounds a noisy line

    def __init__(self, uart):
        self.uart = uart
        self.buffer = b""
        self.frames = 0
        self.errors = 0

    def _read_waiting(self):
        if not hasattr(self.uart, "any"):
            return self.uart.read()
        # any() "may return 1 even if more characters are available" (MicroPython; the board
        # does): reading any() bytes once took 1 byte per camera frame, the K230 RX buffer
        # overflowed and only stale frames came through. Drain until it reports 0 instead.
        chunks, total = [], 0
        while total < self.MAX_READ:
            waiting = self.uart.any()
            if not waiting:
                break
            data = self.uart.read(min(waiting, self.MAX_READ - total))
            if not data:
                break
            chunks.append(data)
            total += len(data)
        return b"".join(chunks) if chunks else None

    def poll(self):
        try:
            data = self._read_waiting()
        except OSError:
            data = None
        if not data:
            return None
        pending = self.buffer + data
        latest = None
        while True:
            end = pending.find(b"\n")
            if end < 0:
                break
            line = pending[:end].rstrip(b"\r")
            pending = pending[end + 1:]
            start = line.rfind(b"$")
            if start < 0:
                continue
            decoded = decode_task(line[start:])
            if decoded is None:
                self.errors += 1
            else:
                latest = decoded
                self.frames += 1
        if len(pending) > self.MAX_PENDING:          # noise without a line end
            start = pending.rfind(b"$")
            pending = pending[start:] if 0 <= start and len(pending) - start <= self.MAX_PENDING else b""
        self.buffer = pending
        return latest
