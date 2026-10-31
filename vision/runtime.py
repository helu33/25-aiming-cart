"""One fresh image produces one result; no replay of a previous correction."""
from protocol import failure_code


def publish_result(sender, result, elapsed_ms, max_age_ms):
    if elapsed_ms < 0 or elapsed_ms > max_age_ms:
        result = {"valid": False, "reason": "frame_too_old", "area_pct": result.get("area_pct")}
    sender.send(result["valid"], result.get("yaw_deg", 0.0), result.get("pitch_deg", 0.0),
                result.get("range_mm", 0.0), max(0, elapsed_ms),
                failure_code(result), result.get("area_pct"))
    return result
