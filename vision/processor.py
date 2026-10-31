"""Calibration gating and frame-to-angle processing, independent of camera/UART."""
import math
from protocol import finite
from geometry import CameraModel, estimate_pose, aim_solution, TargetPoint, circle_point, unit, add, sub, dot, scale
import spot


def calibration_models(data, width, height):
    if not isinstance(data, dict) or data.get("version") != 1:
        raise ValueError("calibration_missing")
    cam, laser = data.get("camera",{}), data.get("laser",{})
    if not isinstance(cam, dict) or not isinstance(laser, dict):
        raise ValueError("invalid_calibration_structure")
    if cam.get("calibrated") is not True:
        raise ValueError("camera_not_calibrated")
    if (cam.get("width"),cam.get("height")) != (width,height):
        raise ValueError("calibration_resolution_mismatch")
    if cam.get("hmirror",False) or cam.get("vflip",False):
        raise ValueError("mirrored_calibration_not_supported")
    values = [cam.get(k) for k in ("fx","fy","cx","cy")]
    distortion = cam.get("distortion",[0,0,0,0,0])
    if not isinstance(distortion, (list,tuple)) or len(distortion) != 5 or any(
            type(v) not in (int,float) or not finite(v) for v in values+list(distortion)):
        raise ValueError("invalid_camera_values")
    camera = CameraModel(width,height,*values,distortion)
    if laser.get("calibrated") is not True:
        raise ValueError("laser_not_calibrated")
    origin, direction = laser.get("origin_mm"), laser.get("direction")
    if not isinstance(origin, (list,tuple)) or not isinstance(direction, (list,tuple)) or len(origin) != 3 or len(direction) != 3 or any(
            type(v) not in (int,float) or not finite(v) for v in list(origin)+list(direction)):
        raise ValueError("invalid_laser_values")
    direction = unit(direction)
    if direction[2] < 0.8:
        raise ValueError("laser_direction_not_forward")
    return camera, tuple(origin), direction


def reprojection_gate(corners, cfg):
    """Largest accepted pose residual (px): the fixed floor, or a fraction of the frame's
    longer diagonal, whichever is larger (the residual scales with the image size)."""
    fixed = cfg.MAX_REPROJECTION_ERROR_PX
    fraction = getattr(cfg, "MAX_REPROJECTION_ERROR_FRACTION", 0.0)
    if not fraction:
        return fixed
    diagonal = max(math.sqrt((corners[0][0]-corners[2][0])**2 + (corners[0][1]-corners[2][1])**2),
                   math.sqrt((corners[1][0]-corners[3][0])**2 + (corners[1][1]-corners[3][1])**2))
    return max(fixed, fraction*diagonal)


class VisionProcessor:
    def __init__(self, cfg, detector, calibration):
        self.cfg, self.detector = cfg, detector
        self.target = TargetPoint(cfg.TARGET_CENTER_MM, cfg.TARGET_WIDTH_MM, cfg.TARGET_HEIGHT_MM)
        self.task = (0, 0.0)                 # ($K task, run progress in laps), see set_task
        self.camera = self.origin = self.direction = None
        self.calibration_error = None
        try:
            if cfg.HMIRROR or cfg.VFLIP:
                raise ValueError("image_mirroring_not_supported")
            self.camera,self.origin,self.direction = calibration_models(calibration,cfg.WIDTH,cfg.HEIGHT)
        except (ValueError,TypeError,KeyError) as exc:
            self.calibration_error = str(exc)
        # Closed loop on the observed spot: a slowly learnt offset added to the
        # calibrated laser direction (the field failure mode is a drifting aim).
        self.spot_cfg = getattr(cfg, "SPOT", None)
        self._correction = (0.0, 0.0, 0.0)
        self._frames = 0
        self._spot_status = "spot_off" if not (self.spot_cfg and self.spot_cfg.get("enabled")) else "spot_wait"

    def set_task(self, task, progress_laps):
        """Circle task from the gimbal ($K): aim at the point of the CIRCLE_RADIUS_MM circle around
        the bullseye for the run progress (laps, one circle per lap); task 0 aims at the bullseye."""
        center = self.cfg.TARGET_CENTER_MM
        if task == 1:
            x, y = circle_point(center, getattr(self.cfg, "CIRCLE_RADIUS_MM", 60.0), progress_laps,
                                getattr(self.cfg, "CIRCLE_DIRECTION", 1))
        else:
            x, y = center
        self.target.set_point(x, y)
        self.task = (task, progress_laps)

    def laser_direction(self):
        if self._correction == (0.0, 0.0, 0.0):
            return self.direction
        return unit(add(self.direction, self._correction))

    def correction_deg(self):
        """(yaw, pitch) of the corrected laser direction relative to the calibration."""
        if self.direction is None or self._correction == (0.0, 0.0, 0.0):
            return (0.0, 0.0)
        d, c = self.direction, self.laser_direction()
        return (math.degrees(math.atan2(c[0], c[2]) - math.atan2(d[0], d[2])),
                math.degrees(math.atan2(c[1], c[2]) - math.atan2(d[1], d[2])))

    def _update_correction(self, gray, corners, pose):
        cfg = self.spot_cfg
        try:
            point, status = spot.find_spot(gray, corners, (self.cfg.TARGET_WIDTH_MM, self.cfg.TARGET_HEIGHT_MM),
                                           self.cfg.TAPE_WIDTH_MM, cfg)
        except Exception as exc:   # unknown ulab/cv2 subset on the board: aim without it
            self._spot_status = "spot_disabled:" + type(exc).__name__
            return
        if point is None:
            self._spot_status = status
            return
        xy = self.camera.normalized(point)
        observed = unit(sub(pose.intersection((0, 0, 0), (xy[0], xy[1], 1.0)), self.origin))
        current = self.laser_direction()
        if math.degrees(math.acos(max(-1.0, min(1.0, dot(observed, current))))) > cfg["gate_deg"]:
            self._spot_status = "spot_gated"      # too far from the prediction: glare, not the laser
            return
        wanted = sub(observed, self.direction)
        step = scale(sub(wanted, self._correction), cfg["alpha"])
        correction = add(self._correction, step)
        limit = math.radians(cfg["max_correction_deg"])
        size = math.sqrt(dot(correction, correction))
        if size > limit:
            correction = scale(correction, limit/size)
        self._correction = correction
        self._spot_status = "spot_ok"

    def process(self, gray):
        observation = self.detector.detect(gray)
        result = {"valid": False, "reason": "no_target", "observation": observation,
                  "det": getattr(self.detector, "last_status", ""),
                  "area_pct": 100.0 * getattr(self.detector, "last_area_fraction", 0.0)}
        if observation is None:
            return result
        if self.calibration_error:
            result["reason"] = self.calibration_error
            return result
        if not observation["confirmed"]:
            result["reason"] = "confirming_target"
            return result
        try:
            pose,reprojection = estimate_pose(observation["corners"],self.camera,
                self.cfg.TARGET_WIDTH_MM,self.cfg.TARGET_HEIGHT_MM,
                reprojection_gate(observation["corners"],self.cfg))
            tilt = math.degrees(math.acos(max(-1.0,min(1.0,pose.normal[2]))))
            if tilt > self.cfg.MAX_TILT_DEG:
                raise ValueError("target_tilt_too_large")
            solution = aim_solution(pose,self.origin,self.laser_direction(),self.target.current())
            if not self.cfg.MIN_DEPTH_MM <= solution["depth_mm"] <= self.cfg.MAX_DEPTH_MM:
                raise ValueError("target_depth_out_of_range")
            if max(abs(solution["yaw_deg"]),abs(solution["pitch_deg"])) > self.cfg.MAX_COMMAND_DEG:
                raise ValueError("angular_error_out_of_range")
            if any(not finite(solution[key]) for key in ("yaw_deg","pitch_deg","depth_mm")):
                raise ValueError("nonfinite_solution")
            result.update(solution)
            result.update({"valid":True,"reason":"tracking","tilt_deg":tilt,
                           "reprojection_px":reprojection})
            self._frames += 1
            cfg = self.spot_cfg
            if (gray is not None and cfg and cfg.get("enabled") and
                    not self._spot_status.startswith("spot_disabled") and self._frames % cfg["every"] == 0):
                self._update_correction(gray, observation["corners"], pose)
            result["spot"] = self._spot_status
            result["correction_deg"] = self.correction_deg()
        except (ValueError,ZeroDivisionError,OverflowError) as exc:
            result["reason"] = str(exc)
        return result
