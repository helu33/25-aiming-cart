"""PC-only: turn laser-calibration photos into laser_samples.json (and calibration.json).

For every photo it finds the black frame corners with the board detector and the laser
spot on the white paper inside the frame, writes a check image with both marked, and
prints the estimated distance so depth coverage is visible. Spots are only searched on
the paper inside the tape, so keep the spot off the black tape.

    py -3.13 vision/tools/extract_laser_samples.py                  # samples + check images
    py -3.13 vision/tools/extract_laser_samples.py --calibrate      # also write calibration.json
"""
import argparse
import json
from pathlib import Path
import sys

import cv2
import numpy as np

VISION = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(VISION))
import config
from detector import FrameDetector
from geometry import CameraModel, estimate_pose, homography, transform
from calibrate import fit_laser, read_json, write_json

# Spot score = colour saturation (max-min channel, works for green, red or violet
# lasers) + half the brightness, both measured above the paper's own median so a tinted
# white balance does not lift the whole sheet.
# The 52 grayscale targetcheck photos without a laser peak at 6.5; the threshold is a
# guess for real colour spots and should be revisited after the first laser photos.
SPOT_MIN_PEAK = 60.0
SPOT_MAX_AREA_PX = 900       # floor; close up the halo blooms past this (1400-1500 px at 0.56 m)
SPOT_MAX_PAPER_FRACTION = 0.05  # ...so the limit is also 5% of the visible paper area
PAPER_MARGIN_MM = 4.0      # stay this far inside the tape so its edge never scores


def find_target(gray):
    """Outer frame corners (TL,TR,BR,BL) or None; same threshold search as the board."""
    height, width = gray.shape[:2]
    detector = FrameDetector(width, height, config.DETECTION,
                             (config.TARGET_WIDTH_MM, config.TARGET_HEIGHT_MM), config.TAPE_WIDTH_MM)
    mean = float(gray.mean())
    probes = [float(config.DETECTION["threshold"])]
    probes += [r*mean for r in config.DETECTION.get("threshold_ratios", ())]
    for threshold in probes:
        found = detector.candidates(gray, threshold)
        if found:
            return max(found, key=lambda c: c["area"])["corners"]
    return None


def paper_mask(shape, corners):
    """Pixels of white paper inside the tape (with a safety margin)."""
    w, h = config.TARGET_WIDTH_MM, config.TARGET_HEIGHT_MM
    inset = config.TAPE_WIDTH_MM + PAPER_MARGIN_MM
    mapping = homography(((0, 0), (w, 0), (w, h), (0, h)), corners)
    inner = [transform(mapping, p) for p in ((inset, inset), (w-inset, inset),
                                              (w-inset, h-inset), (inset, h-inset))]
    mask = np.zeros(shape[:2], dtype=np.uint8)
    cv2.fillConvexPoly(mask, np.rint(inner).astype(np.int32), 255)
    return mask


def find_spot(bgr, corners):
    """Laser spot centre on the paper, or None. Returns spot_px, peak score and area."""
    mask = paper_mask(bgr.shape, corners) > 0
    if not mask.any():
        return None
    b, g, r = (bgr[:, :, i].astype(np.float32) for i in range(3))
    gray = 0.114*b + 0.587*g + 0.299*r
    paper = float(np.median(gray[mask]))
    chroma = np.maximum(np.maximum(b, g), r) - np.minimum(np.minimum(b, g), r)
    chroma = np.maximum(chroma - float(np.median(chroma[mask])), 0)
    score = chroma + 0.5*np.maximum(gray - paper, 0)
    score[~mask] = 0
    peak_index = int(np.argmax(score))
    py, px = divmod(peak_index, score.shape[1])
    peak = float(score[py, px])
    if peak < SPOT_MIN_PEAK:
        return None
    region = (score >= 0.5*peak).astype(np.uint8)
    count, labels = cv2.connectedComponents(region)
    blob = labels == labels[py, px]
    area = int(blob.sum())
    max_area = max(SPOT_MAX_AREA_PX, SPOT_MAX_PAPER_FRACTION*float(mask.sum()))
    if count < 2 or area > max_area:
        return None
    weights = score*blob
    ys, xs = np.indices(score.shape)
    total = float(weights.sum())
    return {"spot_px": (float((xs*weights).sum()/total), float((ys*weights).sum()/total)),
            "peak": peak, "area": area}


def camera_model(calibration):
    cam = calibration["camera"]
    return CameraModel(cam["width"], cam["height"], cam["fx"], cam["fy"],
                       cam["cx"], cam["cy"], cam["distortion"])


def extract(paths, calibration, check_dir=None):
    """Samples for fit_laser plus one report row per photo."""
    camera = camera_model(calibration)
    samples, rows = [], []
    for path in paths:
        row = {"file": Path(path).name, "status": "", "distance_mm": None, "spot_px": None}
        rows.append(row)
        bgr = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if bgr is None:
            row["status"] = "unreadable"
            continue
        height, width = bgr.shape[:2]
        if (width, height) != (camera.width, camera.height):
            row["status"] = "resolution_mismatch"
            continue
        corners = find_target(cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY))
        if corners is None:
            row["status"] = "no_target"
        else:
            try:
                pose, _ = estimate_pose(corners, camera, config.TARGET_WIDTH_MM, config.TARGET_HEIGHT_MM,
                                        config.MAX_REPROJECTION_ERROR_PX)
                row["distance_mm"] = float(np.linalg.norm(pose.camera_point(config.TARGET_CENTER_MM)))
            except ValueError as exc:
                row["status"] = "bad_pose:" + str(exc)
            spot = find_spot(bgr, corners) if not row["status"] else None
            if not row["status"]:
                if spot is None:
                    row["status"] = "no_spot"
                else:
                    row["status"] = "ok"
                    row["spot_px"] = spot["spot_px"]
                    samples.append({"width": width, "height": height,
                                    "corners_px": [list(p) for p in corners],
                                    "spot_px": list(spot["spot_px"]), "source_image": row["file"],
                                    "target_mm": [config.TARGET_WIDTH_MM, config.TARGET_HEIGHT_MM]})
        if check_dir is not None:
            write_check_image(Path(check_dir), row, bgr, corners)
    return samples, rows


def write_check_image(check_dir, row, bgr, corners):
    check_dir.mkdir(parents=True, exist_ok=True)
    image = bgr.copy()
    if corners is not None:
        cv2.polylines(image, [np.rint(corners).astype(np.int32)], True, (0, 255, 0), 1)
        for label, point in zip(("TL", "TR", "BR", "BL"), corners):
            cv2.putText(image, label, tuple(int(v) for v in point), cv2.FONT_HERSHEY_SIMPLEX,
                        0.4, (0, 255, 0), 1)
    if row["spot_px"] is not None:
        x, y = (int(round(v)) for v in row["spot_px"])
        cv2.drawMarker(image, (x, y), (0, 0, 255), cv2.MARKER_CROSS, 24, 1)
    distance = "" if row["distance_mm"] is None else " {:.0f}mm".format(row["distance_mm"])
    cv2.putText(image, row["status"] + distance, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                (0, 0, 255), 2)
    cv2.imwrite(str(check_dir / (Path(row["file"]).stem + "_check.png")), image)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--images", default=str(VISION / "captures/laser"))
    parser.add_argument("--camera", default=str(VISION / "camera.json"))
    parser.add_argument("--checks", default=str(VISION / "captures/laser_check"))
    parser.add_argument("--samples", default=str(VISION / "laser_samples.json"))
    parser.add_argument("--exclude", nargs="*", default=[], help="File names to leave out")
    parser.add_argument("--calibrate", action="store_true", help="Also write calibration.json")
    parser.add_argument("--output", default=str(VISION / "calibration.json"))
    args = parser.parse_args()

    paths = [p for p in sorted(Path(args.images).glob("*.bmp")) if p.name not in args.exclude]
    if not paths:
        raise SystemExit("No .bmp photos in " + args.images)
    calibration = read_json(args.camera)
    samples, rows = extract(paths, calibration, args.checks)

    print("{:<28} {:>9}  {:<10} {}".format("photo", "distance", "status", "spot_px"))
    for row in rows:
        distance = "" if row["distance_mm"] is None else "{:.0f} mm".format(row["distance_mm"])
        spot = "" if row["spot_px"] is None else "({:.1f}, {:.1f})".format(*row["spot_px"])
        print("{:<28} {:>9}  {:<10} {}".format(row["file"], distance, row["status"], spot))
    bins = {"near <750": 0, "mid 750-1250": 0, "far >1250": 0}
    for row in rows:
        if row["status"] == "ok":
            d = row["distance_mm"]
            bins["near <750" if d < 750 else ("mid 750-1250" if d <= 1250 else "far >1250")] += 1
    print("usable samples:", len(samples), " coverage:", bins)
    print("check images:", args.checks, "(green = frame, red cross = spot; delete or --exclude wrong ones)")

    with open(args.samples, "w", encoding="utf-8") as stream:
        json.dump({"samples": samples}, stream, indent=2)
        stream.write("\n")
    print("samples written:", args.samples)

    if args.calibrate:
        result = fit_laser(calibration, samples)
        write_json(args.output, result)   # refuses to overwrite an existing file
        laser = result["laser"]
        print("calibration written:", args.output)
        print("  max residual {:.2f} mm, depth {:.0f}-{:.0f} mm, {} samples".format(
            laser["max_residual_mm"], laser["depth_range_mm"][0], laser["depth_range_mm"][1],
            laser["sample_count"]))


if __name__ == "__main__":
    main()
