"""PC-only calibration helper. Never copy NumPy/desktop cv2 onto the K230."""
import argparse
import json
import math
from pathlib import Path
import sys

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from geometry import CameraModel, estimate_pose, norm, sub, scale, dot, unit


def read_json(path):
    with open(path, encoding="utf-8") as stream:
        return json.load(stream)


def write_json(path, data):
    # Never silently overwrite a previous calibration.
    with open(path, "x", encoding="utf-8") as stream:
        json.dump(data, stream, indent=2, allow_nan=False)
        stream.write("\n")


def calibrate_camera(args):
    if args.cols < 3 or args.rows < 3 or not math.isfinite(args.square_mm) or args.square_mm <= 0:
        raise ValueError("Invalid chessboard dimensions")
    pattern = (args.cols, args.rows)
    objects = np.zeros((args.cols * args.rows, 3), np.float32)
    objects[:, :2] = np.mgrid[0:args.cols, 0:args.rows].T.reshape(-1, 2) * args.square_mm
    object_views, image_views, used, size = [], [], [], None
    for path in sorted(Path(args.images).iterdir()):
        if path.suffix.lower() not in (".bmp", ".png", ".jpg", ".jpeg"):
            continue
        gray = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        if gray is None:
            raise ValueError("Cannot read " + str(path))
        current_size = (gray.shape[1], gray.shape[0])
        if size is not None and size != current_size:
            raise ValueError("Mixed image resolutions")
        size = current_size
        found, corners = cv2.findChessboardCorners(gray, pattern)
        if not found:
            print("Skipped (no chessboard):", path.name)
            continue
        corners = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1),
                                  (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 40, 0.001))
        # OpenCV 5 may return Nx2 where 4.x returned Nx1x2.
        corners = np.asarray(corners, dtype=np.float32).reshape(-1, 1, 2)
        object_views.append(objects.copy())
        image_views.append(corners)
        used.append(path.name)
    if len(used) < 12:
        raise ValueError("Need at least 12 usable views with varied tilt, distance and image position")
    rms, matrix, distortion, rvecs, tvecs = cv2.calibrateCamera(
        object_views, image_views, size, None, None)
    errors = []
    for obj, observed, rv, tv in zip(object_views, image_views, rvecs, tvecs):
        projected, _ = cv2.projectPoints(obj, rv, tv, matrix, distortion)
        delta = observed.reshape(-1, 2) - projected.reshape(-1, 2)
        errors.append(float(np.sqrt(np.mean(np.sum(delta**2, axis=1)))))
    if not np.isfinite(matrix).all() or not np.isfinite(distortion).all() or rms > 1.0 or max(errors) > 2.0:
        raise ValueError("Calibration error too large; RMS={} max view={}".format(rms, max(errors)))
    result = {"version": 1, "camera": {
        "calibrated": True, "width": size[0], "height": size[1], "hmirror": False, "vflip": False,
        "fx": float(matrix[0, 0]), "fy": float(matrix[1, 1]),
        "cx": float(matrix[0, 2]), "cy": float(matrix[1, 2]),
        "distortion": distortion.ravel().tolist(),
        "rms_px": float(rms), "max_view_rms_px": max(errors), "images": used},
        "laser": {"calibrated": False, "origin_mm": [0, -18, 0], "direction": None}}
    write_json(args.output, result)
    print("Camera saved:", args.output, "RMS px:", rms, "Next: measured laser spots.")


def fit_laser(data, records):
    cam = data["camera"]
    if cam.get("calibrated") is not True or cam.get("hmirror", False) or cam.get("vflip", False):
        raise ValueError("Need non-mirrored camera calibration first")
    camera = CameraModel(cam["width"], cam["height"], cam["fx"], cam["fy"],
                         cam["cx"], cam["cy"], cam["distortion"])
    if len(records) < 6:
        raise ValueError("Need >=6 measured spots across at least three distances")
    points = []
    for record in records:
        if (record["width"], record["height"]) != (camera.width, camera.height):
            raise ValueError("Sample resolution mismatch")
        corners, spot = record["corners_px"], record["spot_px"]
        if len(corners) != 4 or len(spot) != 2 or any(len(p) != 2 for p in corners):
            raise ValueError("Need four ordered corners and a measured 2D laser spot")
        if any(not math.isfinite(v) for p in corners+[spot] for v in p):
            raise ValueError("Nonfinite sample")
        if any(not (0 <= p[0] < camera.width and 0 <= p[1] < camera.height) for p in corners+[spot]):
            raise ValueError("Sample outside image")
        # User records physical TL,TR,BR,BL; never guess order during calibration.
        # target_mm: outer frame size of the target in that photo (A4 if omitted).
        width_mm, height_mm = record.get("target_mm", (297.0, 210.0))
        pose, _ = estimate_pose(corners, camera, width_mm, height_mm)
        xy = camera.normalized(spot)
        points.append(pose.intersection((0, 0, 0), (xy[0], xy[1], 1)))
    xyz = np.asarray(points, dtype=np.float64)
    if float(np.ptp(xyz[:, 2])) < 200:
        raise ValueError("Depth span must be >=200 mm; cover the full operating range")
    design = np.column_stack((np.ones(len(xyz)), xyz[:, 2]))
    coeff, _, _, _ = np.linalg.lstsq(design, xyz[:, :2], rcond=None)
    origin = (float(coeff[0, 0]), float(coeff[0, 1]), 0.0)
    direction = unit((float(coeff[1, 0]), float(coeff[1, 1]), 1.0))
    residuals = []
    for p in points:
        delta = sub(p, origin)
        residuals.append(norm(sub(delta, scale(direction, dot(delta, direction)))))
    if max(residuals) > 5.0 or direction[2] < 0.8:
        raise ValueError("Laser fit rejected; max residual mm={}".format(max(residuals)))
    # Origin is the fitted line's intersection with camera z=0, not measured emitter position.
    data["laser"] = {"calibrated": True, "origin_mm": list(origin), "direction": list(direction),
                     "max_residual_mm": max(residuals), "sample_count": len(points),
                     "depth_range_mm": [float(xyz[:, 2].min()), float(xyz[:, 2].max())]}
    return data


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    cam = commands.add_parser("camera", help="Chessboard intrinsics (PC only)")
    cam.add_argument("images")
    cam.add_argument("--cols", type=int, required=True, help="Inner corner columns")
    cam.add_argument("--rows", type=int, required=True, help="Inner corner rows")
    cam.add_argument("--square-mm", type=float, required=True)
    cam.add_argument("--output", default="camera.json")
    laser = commands.add_parser("laser", help="Fit ray to manually measured laser spots")
    laser.add_argument("camera_json")
    laser.add_argument("samples_json")
    laser.add_argument("--output", default="calibration.json")
    args = parser.parse_args()
    if args.command == "camera":
        calibrate_camera(args)
    else:
        result = fit_laser(read_json(args.camera_json), read_json(args.samples_json)["samples"])
        write_json(args.output, result)
        print("Laser saved:", args.output, result["laser"])


if __name__ == "__main__":
    main()
