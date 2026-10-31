"""Laser spot on the white target paper in a grayscale frame (K230 ulab or desktop numpy).

Only array operations both libraries share are used (slice, argmax, mean, clip, sum, max);
no per-pixel Python loops. The spot is the brightest blob inside the paper area, measured
as the intensity-weighted centre of its near-saturated core.
"""
try:
    from ulab import numpy as np
except ImportError:
    import numpy as np
from geometry import homography, transform

FLOAT = getattr(np, "float", None) or np.float32


def _paper_bounds(size, tape, margin):
    w, h = size
    inset = tape + margin
    return inset, inset, w - inset, h - inset


def find_spot(gray, corners, size, tape, opts):
    """Return ((x, y), "spot_ok") or (None, reason)."""
    height, width = gray.shape[0], gray.shape[1]
    w, h = size
    to_image = homography(((0, 0), (w, 0), (w, h), (0, h)), corners)
    x_min, y_min, x_max, y_max = _paper_bounds(size, tape, opts["paper_margin_mm"])
    quad = [transform(to_image, p) for p in ((x_min, y_min), (x_max, y_min), (x_max, y_max), (x_min, y_max))]
    x0 = max(0, int(min(p[0] for p in quad)) + 1)
    x1 = min(width, int(max(p[0] for p in quad)))
    y0 = max(0, int(min(p[1] for p in quad)) + 1)
    y1 = min(height, int(max(p[1] for p in quad)))
    if x1 - x0 < 8 or y1 - y0 < 8:
        return None, "spot_roi_small"

    roi = gray[y0:y1, x0:x1]
    peak_index = int(np.argmax(roi))
    py, px = peak_index // (x1 - x0), peak_index % (x1 - x0)
    peak = float(roi[py, px])
    if peak < opts["min_peak"] or peak - float(np.mean(roi)) < opts["min_contrast"]:
        return None, "spot_dim"

    r = opts["window"]
    wy0, wy1 = max(0, py - r), min(y1 - y0, py + r + 1)
    wx0, wx1 = max(0, px - r), min(x1 - x0, px + r + 1)
    core = np.clip(np.array(roi[wy0:wy1, wx0:wx1], dtype=FLOAT) - (peak - opts["core_drop"]), 0, 255)
    strongest = float(np.max(core))
    border = max(float(np.max(core[0, :])), float(np.max(core[-1, :])),
                 float(np.max(core[:, 0])), float(np.max(core[:, -1])))
    if strongest <= 0 or border > opts["border_fraction"]*strongest:
        return None, "spot_too_large"     # bright area wider than the window: glare, not a spot
    rows, cols = np.sum(core, axis=1), np.sum(core, axis=0)
    total = float(np.sum(rows))
    cy = sum(i*float(rows[i]) for i in range(len(rows)))/total
    cx = sum(j*float(cols[j]) for j in range(len(cols)))/total
    point = (x0 + wx0 + cx, y0 + wy0 + cy)

    to_paper = homography(corners, ((0, 0), (w, 0), (w, h), (0, h)))
    u, v = transform(to_paper, point)
    if not (x_min <= u <= x_max and y_min <= v <= y_max):
        return None, "spot_off_paper"
    return point, "spot_ok"
