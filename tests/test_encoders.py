"""Cross-check both sender examples against the production C decoder."""
import ctypes
import importlib.util
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("vision_protocol", ROOT / "docs/examples/vision_protocol.py")
vision = importlib.util.module_from_spec(spec)
spec.loader.exec_module(vision)

# Use a C wrapper to avoid relying on host struct enum/bool padding in ctypes.
lib = ctypes.CDLL(str(ROOT / "tests/.build/encoders.dll"))
lib.test_parse.argtypes = [ctypes.c_char_p]
lib.test_parse.restype = ctypes.c_int
lib.chassis_encode.argtypes = [ctypes.c_char_p, ctypes.c_size_t, ctypes.c_uint16,
                               ctypes.c_uint, ctypes.c_uint] + [ctypes.c_float] * 7 + [ctypes.c_uint] * 2
lib.chassis_encode.restype = ctypes.c_int
assert vision.crc16_ccitt_false(b"123456789") == 0x29B1
for seq in (0, 1, 65535):
    for valid in (True, False):
        packet = vision.encode_vision(seq, valid, 2.5, -1)
        assert lib.test_parse(packet.rstrip(b"\n")) == 1
for bad in (float("nan"), float("inf"), 181):
    try:
        vision.encode_vision(0, True, bad, 0)
        raise AssertionError("invalid value accepted")
    except ValueError:
        pass
buffer = ctypes.create_string_buffer(192)
for mode in (0, 1):
    for valid in (0, 1):
        length = lib.chassis_encode(buffer, len(buffer), 65535, mode, valid, 90, -3, 2, 0, 0, 45, 0.35, 1, 250)
        assert length == len(buffer.value) and buffer.value.endswith(b"\n")
        assert lib.test_parse(buffer.value.rstrip(b"\n")) == 2
        assert b"$C,3," in buffer.value and b",350,1,250*" in buffer.value
assert lib.chassis_encode(buffer, 8, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0) == -1
assert lib.chassis_encode(buffer, 192, 1, 0, 1, float("nan"), 0, 0, 0, 0, 0, 0, 0, 0) == -1
assert lib.chassis_encode(buffer, 192, 1, 0, 1, 0, 0, 0, 0, 0, 0, 3.5, 0, 0) == -1
assert lib.chassis_encode(buffer, 192, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 2, 0) == -1       # task
assert lib.chassis_encode(buffer, 192, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 10000) == -1   # progress
print("Sender examples passed: Python vision and C chassis round-trip through production decoder.")
