"""Vision-side encoder. Standard library only; pass returned bytes to your serial writer."""
import math


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            crc = ((crc << 1) ^ (0x1021 if crc & 0x8000 else 0)) & 0xFFFF
    return crc


def encode_vision(seq: int, valid: bool, yaw_error_deg: float = 0.0,
                  pitch_error_deg: float = 0.0, range_mm: float = 0.0, age_ms: int = 0) -> bytes:
    """Increment seq for each NEW observation; wrap at 65535. No target: valid=False.
    Version 2 adds range_mm (camera to target point, 0 without target) and age_ms."""
    if not isinstance(seq, int) or not 0 <= seq <= 65535:
        raise ValueError("seq must be an integer in [0, 65535]")
    if not isinstance(valid, bool):
        raise ValueError("valid must be bool")
    if not valid:
        yaw_error_deg = pitch_error_deg = range_mm = 0.0
    if any(not math.isfinite(v) or abs(v) > 180 for v in (yaw_error_deg, pitch_error_deg)):
        raise ValueError("errors must be finite degrees in [-180, 180]")
    if not math.isfinite(range_mm) or not math.isfinite(age_ms):
        raise ValueError("range/age must be finite")
    range_mm = min(20000, max(0, round(range_mm)))
    age_ms = min(1000, max(0, round(age_ms)))
    body = (f"V,2,{seq},{int(valid)},{yaw_error_deg:.3f},{pitch_error_deg:.3f},"
            f"{range_mm},{age_ms}").encode("ascii")
    return b"$" + body + f"*{crc16_ccitt_false(body):04X}\n".encode("ascii")


if __name__ == "__main__":
    # The vision pipeline supplies ANGULAR errors, not raw pixel coordinates.
    # serial_port.write(encode_vision(seq, target_found, yaw_error_deg, pitch_error_deg))
    # seq = (seq + 1) & 0xFFFF
    print(encode_vision(1, True, 2.5, -1.0).decode("ascii"), end="")
    print(encode_vision(2, False).decode("ascii"), end="")
