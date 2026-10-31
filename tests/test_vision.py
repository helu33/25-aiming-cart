"""PC checks: synthetic targets, geometric invariants, fault gating and C wire contract."""
import ctypes
import math
from pathlib import Path
import sys
import unittest
from unittest.mock import patch
import importlib.util
import types
import io
import tempfile
import json
from contextlib import redirect_stdout

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tests/.build/vision_deps"))
sys.path.insert(0, str(ROOT / "vision"))
sys.path.insert(0, str(ROOT / "vision/tools"))
import cv2
import numpy as np
import config
from detector import FrameDetector, contour_points, hierarchy_rows
from geometry import CameraModel, PlanePose, estimate_pose, aim_solution, unit, order_quad, TargetPoint, circle_point
from processor import VisionProcessor, reprojection_gate
from protocol import crc16, encode_vision, VisionSender, decode_task, TaskReceiver
from runtime import publish_result
from calibrate import fit_laser, calibrate_camera
import inspect_frame
import extract_laser_samples


# Synthetic scenes are drawn as the contest A4 target; the field config may describe a
# different practice target, so pin the geometry the tests render and keep the field one.
FIELD_TARGET = (config.TARGET_WIDTH_MM, config.TARGET_HEIGHT_MM, config.TAPE_WIDTH_MM, config.TARGET_CENTER_MM)
config.TARGET_WIDTH_MM, config.TARGET_HEIGHT_MM = 297.0, 210.0
config.TAPE_WIDTH_MM, config.TARGET_CENTER_MM = 18.0, (148.5, 105.0)

CAMERA = CameraModel(640, 480, 600, 600, 320, 240)
PLANE = ((0, 0), (297, 0), (297, 210), (0, 210))


def pose_at(z=1000, x=0, y=0, yaw=0, pitch=0):
    a, b = math.radians(yaw), math.radians(pitch)
    r1 = (math.cos(a), 0, -math.sin(a))
    r2 = (math.sin(a)*math.sin(b), math.cos(b), math.cos(a)*math.sin(b))
    t = tuple((x, y, z)[i] - r1[i]*148.5 - r2[i]*105 for i in range(3))
    return PlanePose(r1, r2, t)


def corners(pose, camera=CAMERA):
    return [camera.project(pose.camera_point(p)) for p in PLANE]


def scene(pose, camera=CAMERA):
    gray = np.full((480, 640), 220, dtype=np.uint8)
    outer = np.rint(corners(pose, camera)).astype(np.int32)
    inner = np.rint([camera.project(pose.camera_point(p)) for p in
                    ((18, 18), (279, 18), (279, 192), (18, 192))]).astype(np.int32)
    cv2.fillConvexPoly(gray, outer, 10)
    cv2.fillConvexPoly(gray, inner, 245)
    return gray


def calibration(camera=CAMERA):
    return {"version": 1, "camera": {"calibrated": True, "width": 640, "height": 480,
            "fx": camera.fx, "fy": camera.fy, "cx": camera.cx, "cy": camera.cy,
            "distortion": list(camera.distortion)},
            "laser": {"calibrated": True, "origin_mm": [0, -18, 0], "direction": [0, 0, 1]}}


class FakeUART:
    def __init__(self, partial=False):
        self.frames = []
        self.partial = partial

    def write(self, frame):
        self.frames.append(frame)
        return 3 if self.partial else len(frame)

    def read(self):                       # machine.UART, timeout 0: nothing received
        return None


class FakeRxUART:
    """K230 machine.UART with timeout 0: read() returns what has arrived, None when nothing."""
    def __init__(self, chunks=()):
        self.chunks = list(chunks)

    def read(self):
        return self.chunks.pop(0) if self.chunks else None


def task_frame(seq, task, progress, version=1):
    body = "K,{},{},{},{}".format(version, seq, task, progress).encode()
    return b"$" + body + "*{:04X}".format(crc16(body)).encode()


class GeometryTests(unittest.TestCase):
    def test_circle_point_starts_on_top_and_turns_counter_clockwise(self):
        c = (157.5, 116.5)
        for p, expected in ((0.0, (157.5, 56.5)), (0.25, (97.5, 116.5)), (0.5, (157.5, 176.5)),
                            (0.75, (217.5, 116.5)), (1.0, (157.5, 56.5))):
            x, y = circle_point(c, 60.0, p)
            self.assertAlmostEqual(x, expected[0], places=6)
            self.assertAlmostEqual(y, expected[1], places=6)
        x, y = circle_point(c, 60.0, 0.25, direction=-1)                  # clockwise: right first
        self.assertAlmostEqual(x, 217.5, places=6)
        for k in range(16):
            x, y = circle_point(c, 60.0, k / 16.0)
            self.assertAlmostEqual(math.hypot(x - c[0], y - c[1]), 60.0, places=6)

    def test_unconverged_undistortion_rejected(self):
        for k in (-0.4, -0.2, 1.2):
            camera=CameraModel(640,480,350,350,320,240,(k,0,0,0,0))
            with self.assertRaisesRegex(ValueError,"did not converge"):
                camera.normalized((30,30))

    def test_nonfinite_intrinsics_rejected_in_shared_geometry(self):
        for bad in (float("nan"),float("inf")):
            with self.assertRaises(ValueError): CameraModel(640,480,bad,600,320,240)
            with self.assertRaises(ValueError): CameraModel(640,480,600,600,320,240,(bad,0,0,0,0))

    def test_pose_round_trip_distance_and_tilt(self):
        camera = CameraModel(640,480,610,590,318,242,(-0.1,0.03,0.001,-0.002,0))
        for z in (500,1000,1500):
            for angle in (-55,0,55):
                p = pose_at(z, 35, -20, angle, 12)
                recovered, error = estimate_pose(corners(p,camera),camera)
                self.assertLess(error,1e-5)
                np.testing.assert_allclose(recovered.t,p.t,atol=1e-4)
                np.testing.assert_allclose(recovered.normal,p.normal,atol=1e-5)

    def test_translation_changes_error_and_laser_hit(self):
        left = aim_solution(pose_at(x=-70), (0,-18,0), (0,0,1), (148.5,105))
        right = aim_solution(pose_at(x=70), (0,-18,0), (0,0,1), (148.5,105))
        self.assertLess(left["yaw_deg"],0)
        self.assertGreater(right["yaw_deg"],0)
        self.assertAlmostEqual(right["error_mm"][0],70)
        self.assertNotEqual(left["hit_mm"],right["hit_mm"])

    def test_vertical_parallax_distance_and_zero(self):
        near = aim_solution(pose_at(500),(0,-18,0),(0,0,1),(148.5,105))
        far = aim_solution(pose_at(1500),(0,-18,0),(0,0,1),(148.5,105))
        self.assertGreater(near["pitch_deg"],far["pitch_deg"])
        zero = aim_solution(pose_at(y=-18),(0,-18,0),(0,0,1),(148.5,105))
        self.assertAlmostEqual(zero["pitch_deg"],0)

    def test_refined_pose_beats_the_homography_on_noisy_far_corners(self):
        import geometry
        import random
        rng=random.Random(7)
        cam=CameraModel(640,480,386,515,318,260,(-0.03,-0.039,0,0.0014,0.115))   # field-like intrinsics
        homography_only=lambda pose,*args,**kwargs: pose
        worse=0
        for z,yaw in ((1400,35),(1500,-40),(1200,20),(900,50),(1600,10)):
            truth=pose_at(z=z,yaw=yaw)
            clean=[cam.project(truth.camera_point(p)) for p in PLANE]
            for _ in range(6):
                noisy=[(u+rng.uniform(-0.8,0.8),v+rng.uniform(-0.8,0.8)) for u,v in clean]
                pose,error=estimate_pose(noisy,cam)
                with patch.object(geometry,"refine_pose",homography_only):
                    _,crude=estimate_pose(noisy,cam,max_error_px=1e9)
                self.assertLessEqual(error,crude+1e-9)
                self.assertLess(error,2.5)                 # the rigid fit absorbs 0.8 px noise
                worse+=error>crude-1e-6
                self.assertLess(abs(pose.t[2]-truth.t[2]),0.03*z)
        self.assertLess(worse,10)                         # usually strictly better
        exact=[cam.project(pose_at(z=1300,yaw=30).camera_point(p)) for p in PLANE]
        self.assertLess(estimate_pose(exact,cam)[1],0.05)

    def test_degenerate_quad_rejected(self):
        with self.assertRaises(ValueError):
            estimate_pose([(1,1)]*4,CAMERA)
        with self.assertRaises(ValueError):
            order_quad([(0,0),(1,1),(2,2),(3,3)])

    def test_future_target_point(self):
        target = TargetPoint()
        target.set_point(208.5,105)
        self.assertEqual(target.current(),(208.5,105))
        with self.assertRaises(ValueError):
            target.set_point(500,0)


class DetectionTests(unittest.TestCase):
    def detector(self):
        return FrameDetector(640,480,config.DETECTION)

    def test_larger_neighbor_does_not_steal_confirmed_track(self):
        detector=self.detector()
        a={"area":10000,"center":(250,240),"corners":((200,190),(300,190),(300,290),(200,290))}
        b={"area":14400,"center":(340,240),"corners":((280,180),(400,180),(400,300),(280,300))}
        with patch.object(detector,"candidates",return_value=[a]):
            for _ in range(3): detector.detect(None)
        with patch.object(detector,"candidates",return_value=[a,b]):
            self.assertEqual(detector.detect(None)["center"],a["center"])
        with patch.object(detector,"candidates",return_value=[b]):
            for _ in range(10): self.assertIsNone(detector.detect(None))
            self.assertFalse(detector.detect(None)["confirmed"])
            self.assertFalse(detector.detect(None)["confirmed"])
            self.assertTrue(detector.detect(None)["confirmed"])

    def test_frame_log_fields_candidates_and_area(self):
        detector=self.detector()
        a={"area":10000,"center":(250,240),"corners":((200,190),(300,190),(300,290),(200,290))}
        b={"area":30720,"center":(340,240),"corners":((280,180),(400,180),(400,300),(280,300))}
        with patch.object(detector,"candidates",return_value=[a]):
            detector.detect(None)
        self.assertEqual(detector.last_candidates,1)
        self.assertAlmostEqual(detector.last_area_fraction,10000/307200)
        with patch.object(detector,"candidates",return_value=[a,b]):
            detector.detect(None)                       # associated with a, not the largest
        self.assertEqual(detector.last_candidates,2)
        self.assertAlmostEqual(detector.last_area_fraction,10000/307200)
        with patch.object(detector,"candidates",return_value=[]), \
             patch.object(detector,"_search_threshold",return_value=[]):
            self.assertIsNone(detector.detect(None))
        self.assertEqual((detector.last_candidates,detector.last_area_fraction),(0,0.0))

    def test_size_jump_at_same_center_requires_reacquisition(self):
        detector=self.detector()
        a={"area":10000,"center":(250,240),"corners":((200,190),(300,190),(300,290),(200,290))}
        b={"area":40000,"center":(250,240),"corners":((150,140),(350,140),(350,340),(150,340))}
        with patch.object(detector,"candidates",return_value=[a]):
            for _ in range(3): detector.detect(None)
        with patch.object(detector,"candidates",return_value=[b]):
            self.assertIsNone(detector.detect(None))
            self.assertEqual(detector.last_status,"association_rejected")

    def test_synthetic_range_tilt_and_consecutive_confirmation(self):
        for z in (500,1000,1500):
            for yaw,pitch in ((0,0),(55,0),(-55,0),(0,45),(30,25)):
                with self.subTest(z=z,yaw=yaw,pitch=pitch):
                    detector = self.detector()
                    gray = scene(pose_at(z,yaw=yaw,pitch=pitch))
                    first = detector.detect(gray)
                    self.assertIsNotNone(first)
                    self.assertFalse(first["confirmed"])
                    detector.detect(gray)
                    self.assertTrue(detector.detect(gray)["confirmed"])

    def test_blank_solid_and_wrong_tape_rejected(self):
        gray = np.full((480,640),245,dtype=np.uint8)
        self.assertIsNone(self.detector().detect(gray))
        cv2.rectangle(gray,(200,150),(440,320),0,-1)
        self.assertIsNone(self.detector().detect(gray))
        cv2.rectangle(gray,(260,210),(380,260),245,-1)
        self.assertIsNone(self.detector().detect(gray))

    def test_brief_miss_keeps_confirmation_long_loss_requires_it(self):
        detector = self.detector()
        gray = scene(pose_at())
        blank = np.full_like(gray,255)
        for _ in range(3): detector.detect(gray)
        self.assertIsNone(detector.detect(blank))
        self.assertEqual(detector.last_status,"no_candidates")
        self.assertTrue(detector.detect(gray)["confirmed"])   # one dropped frame: no re-confirm
        tolerance = config.DETECTION["miss_tolerance"]
        for _ in range(tolerance + 1): self.assertIsNone(detector.detect(blank))
        self.assertFalse(detector.detect(gray)["confirmed"])  # longer loss: confirm again
        self.assertEqual(detector.last_status,"confirming")

    def test_dark_scene_found_by_threshold_search(self):
        detector = self.detector()
        dark = (scene(pose_at()).astype(np.float32)*0.25).astype(np.uint8)  # paper ~55 < 70
        self.assertEqual(detector.threshold, config.DETECTION["threshold"])
        results = [detector.detect(dark) for _ in range(8)]
        self.assertTrue(any(r is not None for r in results))
        self.assertTrue(results[-1]["confirmed"])
        self.assertLess(detector.threshold, config.DETECTION["threshold"])

    def test_cluttered_background_found_without_edges(self):
        # Far target in a busy room: Canny edges push the contour count past the clutter
        # limit, so the plain-threshold variant must be among the search probes.
        gray = scene(pose_at(1500))
        rng = np.random.default_rng(3)
        for _ in range(500):
            x, y = int(rng.integers(0, 600)), int(rng.integers(0, 440))
            if 200 < x < 440 and 150 < y < 330:
                continue
            cv2.rectangle(gray, (x, y), (x+int(rng.integers(3, 30)), y+int(rng.integers(3, 30))),
                          int(rng.integers(150, 200)), 1)
        detector = self.detector()
        self.assertEqual(detector.candidates(gray), [])          # edge pass gives up
        results = [detector.detect(gray) for _ in range(12)]
        self.assertTrue(results[-1] is not None and results[-1]["confirmed"])
        self.assertFalse(detector.use_edges)

    def test_threshold_search_tries_the_nearest_setting_first(self):
        # A miss at the threshold in use first probes the closest alternative with the same
        # edge mode, so a dropout at the edge of a threshold recovers on the next frame.
        detector = self.detector()
        tried = []
        detector.candidates = lambda gray, *args: tried.append(args) or []
        gray = np.full((480, 640), 200, dtype=np.uint8)              # scene mean 200
        detector.detect(gray)
        mean_probes = [r*200.0 for r in config.DETECTION["threshold_ratios"]]
        nearest = min((t for t in mean_probes if abs(t - detector.threshold) >= 3.0),
                      key=lambda t: abs(t - detector.threshold))
        self.assertEqual(tried[0], ())                                 # setting in use
        self.assertAlmostEqual(tried[1][0], nearest)
        self.assertEqual(tried[1][1], detector.use_edges)

    def test_threshold_search_costs_at_most_one_extra_pass_per_frame(self):
        detector = self.detector()
        calls = []
        real = detector.candidates
        def counting(gray, *args):
            calls.append(args)
            return real(gray, *args)
        detector.candidates = counting
        blank = np.full((480,640),245,dtype=np.uint8)
        for _ in range(5):
            calls.clear(); detector.detect(blank)
            self.assertLessEqual(len(calls), 2)

    def test_corner_refinement_improves_subpixel_center(self):
        def antialiased(pose, blur):
            s=8
            def poly(points):
                return np.rint([[((x+0.5)*s-0.5)*16,((y+0.5)*s-0.5)*16] for x,y in points]).astype(np.int32)
            big=np.full((480*s,640*s),220,dtype=np.uint8)
            inner=[CAMERA.project(pose.camera_point(p)) for p in ((18,18),(279,18),(279,192),(18,192))]
            cv2.fillPoly(big,[poly(corners(pose))],10,cv2.LINE_8,4)
            cv2.fillPoly(big,[poly(inner)],245,cv2.LINE_8,4)
            gray=cv2.resize(big,(640,480),interpolation=cv2.INTER_AREA)
            return cv2.GaussianBlur(gray,(5,5),1.2) if blur else gray
        def center(q):
            (ax,ay),(bx,by),(cx,cy),(dx,dy)=q
            ux,uy,vx,vy=cx-ax,cy-ay,dx-bx,dy-by
            s=((bx-ax)*vy-(by-ay)*vx)/(ux*vy-uy*vx)
            return ax+s*ux,ay+s*uy
        errors={False:[],True:[]}
        for z in (500,1000,1500):
            for yaw,pitch,x in ((0,0,0),(35,10,20),(-50,0,-30),(10,40,0),(3,2,5)):
                for blur in (False,True):
                    pose=pose_at(z,x,0,yaw,pitch)
                    gray=antialiased(pose,blur)
                    truth=center(corners(pose))
                    for refine in (False,True):
                        options=dict(config.DETECTION,refine_corners=refine)
                        found=FrameDetector(640,480,options).candidates(gray)
                        self.assertTrue(found)
                        got=center(found[0]["corners"])
                        errors[refine].append(math.hypot(got[0]-truth[0],got[1]-truth[1]))
        self.assertLess(sum(errors[True]),0.75*sum(errors[False]))
        self.assertLess(max(errors[True]),1.0)

    def test_vendor_array_shapes(self):
        pts = np.array([[1,2],[3,4],[5,6],[7,8]])
        self.assertEqual(contour_points(pts),contour_points(pts[:,None,:]))
        h = np.array([[-1,-1,1,-1],[-1,-1,-1,0]])
        self.assertEqual(hierarchy_rows(h),hierarchy_rows(h[None,:,:]))


class ProcessingTests(unittest.TestCase):
    class Observations:
        def __init__(self,pose): self.pose=pose
        def detect(self,gray): return {"corners":corners(self.pose),"confirmed":True}

    def test_calibration_required_and_malformed(self):
        for data in (None,{},"bad",[1],{"version":1,"camera":[]}, {"version":1}, calibration()):
            if isinstance(data,dict) and isinstance(data.get("camera"),dict):
                data["camera"]["fx"] = float("nan")
            processor=VisionProcessor(config,self.Observations(pose_at()),data)
            self.assertFalse(processor.process(None)["valid"])

    def test_resolution_mismatch_and_laser_flag(self):
        data=calibration()
        data["camera"]["width"]=320
        self.assertFalse(VisionProcessor(config,self.Observations(pose_at()),data).process(None)["valid"])
        data=calibration()
        data["laser"]["calibrated"]=False
        self.assertFalse(VisionProcessor(config,self.Observations(pose_at()),data).process(None)["valid"])

    def test_tilt_and_depth_gates(self):
        for pose in (pose_at(yaw=75),pose_at(z=200),pose_at(z=4500)):
            self.assertFalse(VisionProcessor(config,self.Observations(pose),calibration()).process(None)["valid"])

    def test_circle_task_moves_the_aim_point(self):
        processor = VisionProcessor(config, self.Observations(pose_at()), calibration())
        self.assertEqual(processor.target.current(), tuple(config.TARGET_CENTER_MM))
        processor.set_task(1, 0.25)
        expected = circle_point(config.TARGET_CENTER_MM, config.CIRCLE_RADIUS_MM, 0.25,
                                config.CIRCLE_DIRECTION)
        self.assertAlmostEqual(processor.target.current()[0], expected[0], places=6)
        self.assertAlmostEqual(processor.target.current()[1], expected[1], places=6)
        self.assertEqual(processor.task, (1, 0.25))
        processor.set_task(0, 0.6)                                          # bullseye again
        self.assertEqual(processor.target.current(), tuple(config.TARGET_CENTER_MM))
        self.assertEqual(config.CIRCLE_RADIUS_MM, 60.0)

    def test_detection_to_angles(self):
        processor=VisionProcessor(config,FrameDetector(640,480,config.DETECTION),calibration())
        gray=scene(pose_at(1000,30,20))
        for _ in range(3): result=processor.process(gray)
        self.assertTrue(result["valid"],result)
        self.assertGreater(result["yaw_deg"],0)
        self.assertGreater(result["pitch_deg"],0)
        self.assertFalse(processor.process(np.full_like(gray,255))["valid"])


class ReprojectionGateTests(unittest.TestCase):
    def test_gate_scales_with_the_frame_size(self):
        # Field: close to the target the ~1% residual exceeded the fixed 3 px gate.
        cfg = types.SimpleNamespace(MAX_REPROJECTION_ERROR_PX=3.0, MAX_REPROJECTION_ERROR_FRACTION=0.015)
        small = ((100, 100), (220, 100), (220, 190), (100, 190))      # 150 px diagonal
        large = ((50, 50), (370, 50), (370, 290), (50, 290))          # 400 px diagonal
        self.assertEqual(reprojection_gate(small, cfg), 3.0)
        self.assertAlmostEqual(reprojection_gate(large, cfg), 6.0)
        cfg.MAX_REPROJECTION_ERROR_FRACTION = 0.0
        self.assertEqual(reprojection_gate(large, cfg), 3.0)


class SpotCorrectionTests(unittest.TestCase):
    """Closed loop: the observed spot corrects a drifted laser direction."""
    TRUE_DIRECTION = unit((math.tan(math.radians(0.6)), math.tan(math.radians(-1.0)), 1))

    @staticmethod
    def frame(pose, spot_px=None):
        gray = (scene(pose).astype(np.float32)*0.7).astype(np.uint8)   # paper ~171, like the field
        if spot_px is not None:
            centre = (int(round(spot_px[0]*8)), int(round(spot_px[1]*8)))
            cv2.circle(gray, centre, 6*8, 215, -1, cv2.LINE_AA, 3)        # halo
            cv2.circle(gray, centre, 3*8, 255, -1, cv2.LINE_AA, 3)        # saturated core
        return gray

    def true_spot(self, pose):
        return CAMERA.project(pose.intersection((0, -18, 0), self.TRUE_DIRECTION))

    def processor(self):
        return VisionProcessor(config, FrameDetector(640, 480, config.DETECTION), calibration())

    def test_gray_spot_located(self):
        import spot
        pose = pose_at(900, yaw=12)
        found, reason = spot.find_spot(self.frame(pose, self.true_spot(pose)), corners(pose),
                                       (297.0, 210.0), 18.0, config.SPOT)
        self.assertEqual(reason, "spot_ok")
        self.assertLess(math.dist(found, self.true_spot(pose)), 0.5)

    def test_correction_converges_to_the_real_laser(self):
        pose = pose_at(1000, yaw=10)
        gray = self.frame(pose, self.true_spot(pose))
        processor = self.processor()
        for _ in range(80):
            result = processor.process(gray)
        self.assertTrue(result["valid"])
        truth = aim_solution(pose, (0, -18, 0), self.TRUE_DIRECTION, (148.5, 105))
        self.assertAlmostEqual(result["yaw_deg"], truth["yaw_deg"], delta=0.1)
        self.assertAlmostEqual(result["pitch_deg"], truth["pitch_deg"], delta=0.1)
        self.assertEqual(result["spot"], "spot_ok")

    def test_no_spot_keeps_correction_and_glare_is_gated(self):
        pose = pose_at(1000)
        processor = self.processor()
        for _ in range(40):
            processor.process(self.frame(pose))                     # laser off: nothing learned
        self.assertEqual(processor.correction_deg(), (0.0, 0.0))
        glare = CAMERA.project(pose.camera_point((148.5 + 90, 105 - 60)))  # far from prediction
        for _ in range(40):
            result = processor.process(self.frame(pose, glare))
        self.assertEqual(processor.correction_deg(), (0.0, 0.0))
        self.assertEqual(result["spot"], "spot_gated")

    def test_spot_failure_never_breaks_aiming(self):
        import spot
        pose = pose_at(1000)
        processor = self.processor()
        with patch.object(spot, "find_spot", side_effect=TypeError("ulab missing feature")):
            for _ in range(10):
                result = processor.process(self.frame(pose, self.true_spot(pose)))
        self.assertTrue(result["valid"])
        self.assertTrue(result["spot"].startswith("spot_disabled"))
        self.assertEqual(processor.correction_deg(), (0.0, 0.0))

    def test_spot_runs_only_every_nth_frame(self):
        import spot
        pose = pose_at(1000)
        processor = self.processor()
        gray = self.frame(pose, self.true_spot(pose))
        calls = []
        real = spot.find_spot
        with patch.object(spot, "find_spot", side_effect=lambda *a: calls.append(1) or real(*a)):
            for _ in range(30):
                processor.process(gray)
        self.assertLessEqual(len(calls), 30 // config.SPOT["every"] + 1)


class TransportTests(unittest.TestCase):
    def test_golden_and_c_decoder(self):
        self.assertEqual(crc16(b"123456789"),0x29B1)
        self.assertEqual(encode_vision(1,True,2.5,-1,812.4,43),b'$V,2,1,1,2.500,-1.000,812,43*1155\n')
        self.assertEqual(encode_vision(2,False),b'$V,2,2,0,0.000,0.000,0,0*DCBB\n')
        dll=ROOT/"tests/.build/encoders.dll"
        self.assertTrue(dll.exists(),"Run tests/run_tests.ps1 to build the production decoder bridge")
        bridge=ctypes.CDLL(str(dll))
        bridge.test_parse.argtypes=[ctypes.c_char_p]
        bridge.test_parse.restype=ctypes.c_int
        for seq in (0,1,65535):
            for valid in (True,False):
                self.assertEqual(bridge.test_parse(encode_vision(seq,valid,2.5,-1,812,43).rstrip(b"\n")),1)

    def test_task_frames_from_the_gimbal(self):
        self.assertEqual(decode_task(task_frame(7, 1, 250)), (1, 0.25))
        self.assertEqual(decode_task(task_frame(65535, 0, 9999)), (0, 9.999))
        bad = task_frame(7, 1, 250)
        for frame in (bad[:-1] + b"0" if bad[-1:] != b"0" else bad[:-1] + b"1",   # CRC
                      task_frame(7, 1, 250, version=2), task_frame(7, 2, 0), task_frame(7, 0, 10000),
                      b"$V,1,7,1,250*0000", b"garbage", b""):
            self.assertIsNone(decode_task(frame), frame)
        # the production C encoder (gimbal) against this decoder (K230)
        dll = ROOT / "tests/.build/encoders.dll"
        self.assertTrue(dll.exists(), "Run tests/run_tests.ps1 to build the production decoder bridge")
        bridge = ctypes.CDLL(str(dll))
        bridge.GimbalProtocol_EncodeTask.argtypes = [ctypes.c_char_p, ctypes.c_size_t, ctypes.c_uint16,
                                                     ctypes.c_uint, ctypes.c_uint]
        bridge.GimbalProtocol_EncodeTask.restype = ctypes.c_int
        buffer = ctypes.create_string_buffer(40)
        for seq, task, progress in ((0, 0, 0), (12, 1, 2500), (65535, 1, 9999)):
            n = bridge.GimbalProtocol_EncodeTask(buffer, len(buffer), seq, task, progress)
            self.assertGreater(n, 0)
            self.assertEqual(decode_task(buffer.value.rstrip(b"\n")), (task, progress / 1000.0))

    def test_task_receiver_never_blocks_and_keeps_the_latest(self):
        a, b = task_frame(1, 1, 100), task_frame(2, 1, 150)
        receiver = TaskReceiver(FakeRxUART([None, b"xx\x01" + a[:6], a[6:] + b"\r\n" + b[:9], b[9:] + b"\n",
                                            task_frame(3, 0, 0) + b"\n" + task_frame(4, 1, 400) + b"\n"]))
        self.assertIsNone(receiver.poll())                               # nothing arrived
        self.assertIsNone(receiver.poll())                               # half a frame
        self.assertEqual(receiver.poll(), (1, 0.1))                      # rest of a, part of b
        self.assertEqual(receiver.poll(), (1, 0.15))
        self.assertEqual(receiver.poll(), (1, 0.4))                      # two in one read: latest
        self.assertIsNone(receiver.poll())
        self.assertEqual(receiver.frames, 4)
        # with any(): it "may return 1 even if more characters are available" (MicroPython docs;
        # the board does): drain until it says 0, so a frame is never read 1 byte per camera frame
        class AnyUART:
            def __init__(self, data):
                self.data, self.reads = bytearray(data), 0
            def any(self):
                return 1 if self.data else 0
            def read(self, n=None):
                self.reads += 1
                n = len(self.data) if n is None else n
                out, self.data = bytes(self.data[:n]), self.data[n:]
                return out or None
        frames = b"".join(task_frame(k, 1, 100 * k) + b"\n" for k in range(1, 9))     # 8 frames waiting
        uart = AnyUART(frames)
        receiver = TaskReceiver(uart)
        self.assertEqual(receiver.poll(), (1, 0.8))                      # all read, newest kept
        self.assertEqual((receiver.frames, len(uart.data)), (8, 0))
        reads = uart.reads
        self.assertIsNone(receiver.poll())
        self.assertEqual(uart.reads, reads)                               # nothing waiting: no read
        uart = AnyUART(b"B" * 5000)                                       # endless noise: bounded
        TaskReceiver(uart).poll()
        self.assertLessEqual(5000 - len(uart.data), TaskReceiver.MAX_READ)
        receiver = TaskReceiver(FakeRxUART([b"A" * 500, b"\n" + task_frame(5, 1, 5) + b"\n"]))
        self.assertIsNone(receiver.poll())                               # overlong noise dropped
        self.assertLess(len(receiver.buffer), 64)
        self.assertEqual(receiver.poll(), (1, 0.005))

    def test_wrap_partial_write_and_expiry(self):
        uart=FakeUART()
        sender=VisionSender(uart)
        sender.seq=65535
        sender.send(True,1,2)
        self.assertEqual(sender.seq,0)
        result=publish_result(sender,{"valid":True,"yaw_deg":12,"pitch_deg":-5},201,200)
        self.assertFalse(result["valid"])
        self.assertIn(b",0,0.000,0.000,0,201,8,0*",uart.frames[-1])   # too old: invalid, age kept
        publish_result(sender,{"valid":True,"yaw_deg":1,"pitch_deg":2,"range_mm":805.6,"area_pct":1.234},37,200)
        self.assertIn(b"$V,3,",uart.frames[-1])
        self.assertIn(b",1,1.000,2.000,806,37,0,123*",uart.frames[-1])
        partial=VisionSender(FakeUART(True))
        self.assertFalse(partial.send(True,1,1))
        self.assertEqual(partial.uart.frames[-1],b"\n")

    def test_failure_codes_and_version_3_fields(self):
        import protocol as P
        cases=(({"valid":True},P.CODE_VALID),
               ({"valid":False,"reason":"no_target","det":"no_candidates"},P.CODE_NO_RECTANGLE),
               ({"valid":False,"reason":"no_target","det":"association_rejected"},P.CODE_ASSOCIATION),
               ({"valid":False,"reason":"no_target","det":"confirming"},P.CODE_CONFIRMING),
               ({"valid":False,"reason":"confirming_target","det":"confirming"},P.CODE_CONFIRMING),
               ({"valid":False,"reason":"pose reprojection error 4.1>3.0px"},P.CODE_REPROJECTION),
               ({"valid":False,"reason":"target_tilt_too_large"},P.CODE_TILT),
               ({"valid":False,"reason":"target_depth_out_of_range"},P.CODE_DEPTH),
               ({"valid":False,"reason":"angular_error_out_of_range"},P.CODE_ANGLE),
               ({"valid":False,"reason":"frame_too_old"},P.CODE_TOO_OLD),
               ({"valid":False,"reason":"non-convex quadrilateral"},P.CODE_OTHER))
        for result,code in cases:
            self.assertEqual(P.failure_code(result),code,result)
        frame=encode_vision(5,False,code=P.CODE_ASSOCIATION,area_pct=150.0)
        self.assertTrue(frame.startswith(b"$V,3,5,0,0.000,0.000,0,0,2,10000*"))   # area clamped
        self.assertTrue(encode_vision(5,False).startswith(b"$V,2,"))                # no code: v2
        with self.assertRaises(ValueError): encode_vision(0,False,code=100)
        processor=VisionProcessor(config,FrameDetector(640,480,config.DETECTION),calibration())
        result=processor.process(np.full((480,640),220,dtype=np.uint8))
        self.assertEqual((result["det"],result["area_pct"]),("no_candidates",0.0))
        self.assertEqual(P.failure_code(result),P.CODE_NO_RECTANGLE)

    def test_nonfinite_rejected(self):
        for value in (float("nan"),float("inf"),181):
            with self.assertRaises(ValueError): encode_vision(0,True,value,0)


class LaserCalibrationTests(unittest.TestCase):
    def records(self):
        records=[]
        origin=(2,-18,0)
        direction=unit((0.004,-0.003,1))
        for z in (500,1000,1500):
            for tilt in (-15,20):
                pose=pose_at(z,yaw=tilt)
                spot=CAMERA.project(pose.intersection(origin,direction))
                records.append({"width":640,"height":480,"corners_px":corners(pose),"spot_px":spot})
        return records,origin,direction

    def test_fit_measured_ray(self):
        records,origin,direction=self.records()
        fitted=fit_laser(calibration(),records)["laser"]
        np.testing.assert_allclose(fitted["origin_mm"],origin,atol=1e-6)
        np.testing.assert_allclose(fitted["direction"],direction,atol=1e-6)

    def test_reject_single_depth_or_outlier(self):
        records,_,_=self.records()
        with self.assertRaises(ValueError): fit_laser(calibration(),[records[0]]*6)
        records[0]["spot_px"]=(400,330)
        with self.assertRaises(ValueError): fit_laser(calibration(),records)


class LaserSampleExtractionTests(unittest.TestCase):
    ORIGIN = (2, -18, 0)
    DIRECTION = unit((0.004, -0.003, 1))

    def photo(self, pose, spot_px=None, saturated=False):
        bgr = cv2.cvtColor(scene(pose), cv2.COLOR_GRAY2BGR)
        if spot_px is not None:
            # Draw at 8x sub-pixel precision so the ground truth is not rounded.
            center = (int(round(spot_px[0]*8)), int(round(spot_px[1]*8)))
            cv2.circle(bgr, center, 5*8, (255, 90, 190), -1, cv2.LINE_AA, 3)   # violet halo
            if saturated:
                cv2.circle(bgr, center, 2*8, (255, 255, 255), -1, cv2.LINE_AA, 3)
        return bgr

    def test_violet_spot_located(self):
        pose = pose_at(800, yaw=15)
        spot = CAMERA.project(pose.intersection(self.ORIGIN, self.DIRECTION))
        found = extract_laser_samples.find_spot(self.photo(pose, spot), corners(pose))
        self.assertIsNotNone(found)
        self.assertLess(math.dist(found["spot_px"], spot), 0.6)

    def test_green_spot_located(self):
        pose = pose_at(900, yaw=-10)
        spot = CAMERA.project(pose.intersection(self.ORIGIN, self.DIRECTION))
        bgr = cv2.cvtColor(scene(pose), cv2.COLOR_GRAY2BGR)
        center = (int(round(spot[0]*8)), int(round(spot[1]*8)))
        cv2.circle(bgr, center, 5*8, (60, 255, 80), -1, cv2.LINE_AA, 3)
        found = extract_laser_samples.find_spot(bgr, corners(pose))
        self.assertIsNotNone(found)
        self.assertLess(math.dist(found["spot_px"], spot), 0.6)

    def test_saturated_white_core_located(self):
        pose = pose_at(600)
        spot = CAMERA.project(pose.intersection(self.ORIGIN, self.DIRECTION))
        found = extract_laser_samples.find_spot(self.photo(pose, spot, saturated=True), corners(pose))
        self.assertIsNotNone(found)
        self.assertLess(math.dist(found["spot_px"], spot), 0.6)

    def test_large_close_range_halo_located(self):
        # At ~0.5 m a green spot blooms to well over 900 px; the area limit must scale.
        pose = pose_at(450)
        spot = CAMERA.project(pose.intersection(self.ORIGIN, self.DIRECTION))
        bgr = cv2.cvtColor(scene(pose), cv2.COLOR_GRAY2BGR)
        center = (int(round(spot[0]*8)), int(round(spot[1]*8)))
        cv2.circle(bgr, center, 24*8, (90, 230, 110), -1, cv2.LINE_AA, 3)
        cv2.circle(bgr, center, 8*8, (200, 255, 210), -1, cv2.LINE_AA, 3)
        found = extract_laser_samples.find_spot(bgr, corners(pose))
        self.assertIsNotNone(found)
        self.assertGreater(found["area"], 900)
        self.assertLess(math.dist(found["spot_px"], spot), 1.0)

    def test_no_spot_rejected(self):
        pose = pose_at(1000)
        self.assertIsNone(extract_laser_samples.find_spot(self.photo(pose), corners(pose)))

    def test_photos_round_trip_to_laser_ray(self):
        with tempfile.TemporaryDirectory(dir=ROOT/"tests/.build") as folder:
            for z in (500, 1000, 1500):
                for tilt in (-15, 20):
                    pose = pose_at(z, yaw=tilt)
                    spot = CAMERA.project(pose.intersection(self.ORIGIN, self.DIRECTION))
                    cv2.imwrite(str(Path(folder)/"frame_{}_{}.bmp".format(z, tilt+20)), self.photo(pose, spot))
            samples, rows = extract_laser_samples.extract(sorted(Path(folder).glob("*.bmp")), calibration())
        self.assertEqual(len(samples), 6)
        self.assertTrue(all(row["status"] == "ok" for row in rows))
        laser = fit_laser(calibration(), samples)["laser"]
        np.testing.assert_allclose(laser["direction"], self.DIRECTION, atol=2e-3)
        np.testing.assert_allclose(laser["origin_mm"], self.ORIGIN, atol=2.0)

    def test_real_target_photos_without_laser_give_no_spot(self):
        photos = sorted((ROOT/"vision/captures/targetcheck").glob("*.bmp"))
        if not photos:
            self.skipTest("targetcheck photos not present")
        width, height, tape, centre = FIELD_TARGET
        with patch.multiple(config, TARGET_WIDTH_MM=width, TARGET_HEIGHT_MM=height,
                            TAPE_WIDTH_MM=tape, TARGET_CENTER_MM=centre):
            _, rows = extract_laser_samples.extract(photos, calibration(CameraModel(
                640, 480, 385.98, 514.74, 318.44, 259.94, (-0.0303, -0.0390, -0.0005, 0.0014, 0.1147))))
        self.assertGreater(sum(row["status"] != "no_target" for row in rows), 40)
        false_spots = [row["file"] for row in rows if row["status"] == "ok"]
        self.assertEqual(false_spots, [])


class MainLifecycleTests(unittest.TestCase):
    def run_board_stub(self,capture=False,slow=False,preview=False,grayscale=True,frame_log_dir=None):
        spec=importlib.util.spec_from_file_location("board_main",ROOT/"vision/main.py")
        main=importlib.util.module_from_spec(spec)
        spec.loader.exec_module(main)
        events=[]
        clock=[0]
        uart=FakeUART()
        uart.deinit=lambda: events.append("uart_deinit")
        class UART:
            UART1=1; EIGHTBITS=8; PARITY_NONE=None; STOPBITS_ONE=1
            def __new__(cls,*args,**kwargs): return uart
        class FPIOA:
            UART1_TXD=1; UART1_RXD=2
            def set_function(self,*args): pass
        class Image:
            def to_grayscale(self):
                events.append("to_grayscale")
                return self
            def to_numpy_ref(self): return scene(pose_at())
            def save(self,path): events.append("save")
            def to_rgb888(self):
                events.append("preview_copy")
                return PreviewImage()
        class PreviewImage:
            def draw_line(self,*args,**kwargs): events.append("draw_line")
        class Display:
            VIRT=0
            @staticmethod
            def init(kind,**kwargs):
                assert kind==Display.VIRT and kwargs["to_ide"]
                events.append("display_init")
            @staticmethod
            def show_image(img):
                assert isinstance(img,PreviewImage)
                events.append("show_image")
            @staticmethod
            def deinit(): events.append("display_deinit")
        class Sensor:
            RGB565=2; GRAYSCALE=1
            def __init__(self): self.count=0
            def reset(self): pass
            def set_framesize(self,**kwargs): pass
            def set_pixformat(self,fmt): events.append(("pixformat",fmt))
            def set_hmirror(self,*args): pass
            def set_vflip(self,*args): pass
            def run(self): events.append("sensor_run")
            def stop(self): events.append("sensor_stop")
            def snapshot(self):
                self.count+=1
                if self.count>4: raise KeyboardInterrupt()
                clock[0]+=250 if slow else 20
                return Image()
        media=types.SimpleNamespace(init=lambda:events.append("media_init"),
                                    deinit=lambda:events.append("media_deinit"))
        fake_time=types.SimpleNamespace(ticks_ms=lambda:clock[0],ticks_diff=lambda a,b:a-b,
                                        sleep_ms=lambda ms:None)
        modules={"machine":types.SimpleNamespace(UART=UART,FPIOA=FPIOA),
                 "media":types.ModuleType("media"),"media.sensor":types.SimpleNamespace(Sensor=Sensor),
                 "media.display":types.SimpleNamespace(Display=Display),
                 "media.media":types.SimpleNamespace(MediaManager=media)}
        output=io.StringIO()
        with patch.dict(sys.modules,modules), patch.object(main,"time",fake_time), \
             patch.object(main.os,"uname",return_value="K230 test stub",create=True), \
             patch.object(main,"load_calibration",return_value=calibration()), \
             patch.object(main.os,"exitpoint",create=True), patch.object(main.os,"mkdir"), \
             patch.object(main,"capture_path",return_value="unused.bmp"), \
             patch.object(config,"CAPTURE_INTERVAL_MS",1 if capture else 0), \
             patch.object(config,"IDE_PREVIEW",preview), \
             patch.object(config,"SENSOR_GRAYSCALE",grayscale), \
             patch.object(config,"LOG_INTERVAL_MS",1), \
             patch.object(config,"FRAME_LOG",frame_log_dir is not None), \
             patch.object(config,"FRAME_LOG_FLUSH_MS",30), \
             patch.object(main,"CAPTURE_DIRECTORY",str(frame_log_dir or "unused")), \
             redirect_stdout(output):
            main.main()
        self.board_log=output.getvalue()
        self.assertLess(events.index("media_init"),events.index("sensor_run"))
        self.assertLess(events.index("sensor_stop"),events.index("media_deinit"))
        self.assertEqual(events[-1],"uart_deinit")
        self.assertEqual(uart.frames[-1].split(b",")[3],b"0")
        return uart.frames,events

    def test_headless_normal_shutdown(self):
        frames,_=self.run_board_stub()
        self.assertTrue(any(f.split(b",")[3]==b"1" for f in frames))

    def test_frame_log_records_every_frame_and_flushes_on_stop(self):
        import framelog
        with tempfile.TemporaryDirectory() as d:
            self.run_board_stub(frame_log_dir=d)
            logs=list(Path(d).glob("framelog_*.csv"))
            self.assertEqual(len(logs),1)
            lines=logs[0].read_text().strip().splitlines()
        self.assertEqual(lines[0],framelog.HEADER)
        self.assertEqual(lines.count(framelog.HEADER),1)
        self.assertEqual(len(lines),1+4)                       # the stub delivers 4 frames
        self.assertIn("Frame log lines: 5",self.board_log)

    def test_grayscale_capture_skips_conversion_and_rgb_fallback_converts(self):
        frames,events=self.run_board_stub()
        self.assertIn(("pixformat",1),events)
        self.assertNotIn("to_grayscale",events)
        frames,events=self.run_board_stub(grayscale=False)
        self.assertIn(("pixformat",2),events)
        self.assertIn("to_grayscale",events)
        self.assertTrue(any(f.split(b",")[3]==b"1" for f in frames))

    def test_capture_mode_never_sends_correction(self):
        frames,events=self.run_board_stub(capture=True)
        self.assertIn("save",events)
        self.assertTrue(all(f.split(b",")[3]==b"0" for f in frames))
        self.assertIn("reason=calibration_capture",self.board_log)
        self.assertIn("det=confirmed seen=1/1 source=tracking",self.board_log)
        self.assertIn("preview_ms=0 save_ms=0",self.board_log)

    def test_slow_frame_never_sends_correction(self):
        frames,_=self.run_board_stub(slow=True)
        self.assertTrue(all(f.split(b",")[3]==b"0" for f in frames))

    def test_preview_uses_virtual_display_and_copy_and_is_throttled(self):
        _,events=self.run_board_stub(preview=True)
        self.assertLess(events.index("display_init"),events.index("media_init"))
        self.assertEqual(events.count("show_image"),1)  # Four 20 ms frames, 100 ms preview limit.
        self.assertEqual(events.count("preview_copy"),1)
        self.assertEqual(events.count("draw_line"),6)  # Outer frame and center cross.
        self.assertLess(events.index("display_deinit"),events.index("media_deinit"))
        _,disabled=self.run_board_stub()
        self.assertNotIn("display_init",disabled)


class DesktopToolTests(unittest.TestCase):
    def test_inspect_frame_exports_sample_and_warp(self):
        with tempfile.TemporaryDirectory(dir=ROOT/"tests/.build") as folder:
            folder=Path(folder)
            source=folder/"frame.bmp"
            cv2.imwrite(str(source),scene(pose_at(yaw=35)))
            args=["inspect_frame.py",str(source),"--output-prefix",str(folder/"check")]
            with patch.object(sys,"argv",args),redirect_stdout(io.StringIO()):
                inspect_frame.main()
            sample=json.loads((folder/"check_sample.json").read_text())
            self.assertEqual(len(sample["corners_px"]),4)
            self.assertEqual(sample["spot_px"],[None,None])
            warped=cv2.imread(str(folder/"check_rectified.png"))
            self.assertEqual(warped.shape[:2],(421,595))

    def test_camera_calibration_from_rendered_chessboards(self):
        with tempfile.TemporaryDirectory(dir=ROOT/"tests/.build") as folder:
            folder=Path(folder)
            board=np.zeros((280,400),dtype=np.uint8)
            for y in range(7):
                for x in range(10):
                    if (x+y)%2: board[y*40:(y+1)*40,x*40:(x+1)*40]=255
            for i in range(16):
                pose=pose_at(550+(i%4)*80,x=(i%3-1)*40,y=(i%4-1.5)*20,
                             yaw=-30+(i%4)*20,pitch=-24+(i//4)*16)
                target=[CAMERA.project(pose.camera_point(p)) for p in ((0,0),(200,0),(200,140),(0,140))]
                transform=cv2.getPerspectiveTransform(np.float32([[0,0],[400,0],[400,280],[0,280]]),np.float32(target))
                image=cv2.warpPerspective(board,transform,(640,480),borderValue=180)
                cv2.imwrite(str(folder/("view%02d.png"%i)),image)
            args=types.SimpleNamespace(cols=9,rows=6,square_mm=20,images=str(folder),output=str(folder/"camera.json"))
            with redirect_stdout(io.StringIO()): calibrate_camera(args)
            result=json.loads(Path(args.output).read_text())
            self.assertTrue(result["camera"]["calibrated"])
            self.assertFalse(result["laser"]["calibrated"])
            self.assertAlmostEqual(result["camera"]["fx"],600,delta=20)
            self.assertAlmostEqual(result["camera"]["fy"],600,delta=20)


if __name__ == "__main__":
    unittest.main(verbosity=2)
