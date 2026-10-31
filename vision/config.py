"""Yahboom GC2093 configuration. Copy this whole directory to /sdcard/k230_vision/."""
WIDTH = 640
HEIGHT = 480
HMIRROR = False
VFLIP = False
SENSOR_GRAYSCALE = True   # False: capture RGB565 and convert each frame (slower).
UART_BAUD = 921600
UART_TX_PIN = 9
UART_RX_PIN = 10

# No physical LCD. VIRT is optional for firmware requiring a display target.
USE_VIRTUAL_DISPLAY = False
IDE_PREVIEW = False       # USB CanMV IDE framebuffer; no physical LCD required.
PREVIEW_INTERVAL_MS = 100 # At most 10 preview frames/s; USB adds loop latency.
STARTUP_SETTLE_MS = 800
# Motion blur: the camera turns with the gimbal (20+ deg/s while the car drives past the
# target) and the frame stopped being found (det=no_candidates). None keeps auto exposure;
# a number fixes the exposure in microseconds (try 8000, then 4000; raise ANALOG_GAIN if the
# image gets too dark for the threshold). Needs CanMV K230 firmware >= v1.6
# (sensor.auto_exposure / exposure / again); on older firmware a warning is printed and the
# camera stays on auto. The start-up log always prints the exposure in use.
EXPOSURE_US = None
ANALOG_GAIN = None
LOOP_YIELD_MS = 0        # snapshot() already blocks for the next frame.
GC_INTERVAL_FRAMES = 10  # Set 1 if the board reports memory/buffer errors.
MAX_RESULT_AGE_MS = 200  # Send invalid if acquisition+processing took longer.
LOG_INTERVAL_MS = 1000
# Per-frame log on the SD card (framelog.py) to find out why frames drop while driving;
# deploy.ps1 -FrameLog switches it on. The write blocks the loop for a few ms every
# FRAME_LOG_FLUSH_MS, so leave it off for normal runs.
FRAME_LOG = False
FRAME_LOG_FLUSH_MS = 10000
CALIBRATION_FILE = "calibration.json"  # Resolved relative to the board script directory.

# Optional calibration images, disabled by default. Saves BMP; disables correction.
CAPTURE_INTERVAL_MS = 0
CAPTURE_DIRECTORY = "captures"

# Practice target measured 2026-10-18: black frame outer edge 315 x 233 mm, tape 20 mm.
# With A4 values (297 x 210, 18) the pose residual was 4.2-4.4 px median and most frames
# were rejected; with the measured size it is 1.6-1.7 px. The contest target is A4 with
# 18 mm tape on the paper edge: switch to 297.0 / 210.0 / 18.0 / (148.5, 105.0) for it.
TARGET_WIDTH_MM = 315.0
TARGET_HEIGHT_MM = 233.0
TAPE_WIDTH_MM = 20.0
TARGET_CENTER_MM = (157.5, 116.5)
# Task 3 (circle): with $K task 1 from the gimbal the aim point runs around a circle of this
# radius on the target, centred on TARGET_CENTER_MM, one circle per lap (the chassis' run
# progress). Starts at the top; 1 = counter-clockwise as seen facing the target, -1 = clockwise.
CIRCLE_RADIUS_MM = 60.0
CIRCLE_DIRECTION = 1
LASER_APPROX_ORIGIN_MM = (0.0, -18.0, 0.0)  # Camera axes: right, down, forward.

DETECTION = {
    "min_area_fraction": 0.008,
    "max_area_fraction": 0.92,
    "threshold": 70,    # Starting value, tuned on targetcheck photos.
    # When the tape is not found, one extra threshold is tried per frame: the value
    # above, then these fractions of the scene mean. On the 53 targetcheck photos this
    # keeps detection at 49-52/53 for 0.7x-1.6x brightness (fixed 70: 41-50, 0 at 0.5x).
    "threshold_ratios": (0.4, 0.55, 0.7, 0.85),
    "miss_tolerance": 2,  # Dropped frames that keep the confirmation (no 3-frame restart).
    "blur_size": 3,
    "close_size": 3,
    "canny_low": 60,
    "canny_high": 150,
    "use_edges": True,
    "approx_fraction": 0.02,
    "min_edge_pixels": 10.0,
    "min_inner_area_ratio": 0.48,
    "max_inner_area_ratio": 0.88,
    "inner_inset_tolerance_mm": 10.0,
    "border_margin_pixels": 2,
    "max_contours": 256,
    "max_jump_fraction": 0.20,
    "max_corner_shift_fraction": 0.35,  # Corner RMS displacement / sqrt(previous area).
    "max_area_change_ratio": 1.8,
    "confirm_frames": 3,
    "refine_corners": True,        # Line-fit outer corners from the contour.
    "corner_refine_max_px": 3.0,   # Larger moves keep the approxPolyDP corner.
}

# Closed loop on the observed laser spot (grayscale, paper area only, every Nth tracked
# frame): learns an offset to the calibrated laser direction. A spot more than gate_deg
# away from the prediction is ignored (glare); the learnt offset is capped.
SPOT = {
    "enabled": True,
    "every": 5,               # frames between spot measurements
    "alpha": 0.25,            # low-pass gain per measurement (~1 s time constant at 23 fps)
    "min_peak": 200,          # spot core must reach this grey level ...
    "min_contrast": 45,       # ... and be this much above the paper mean
    "core_drop": 25,          # core = pixels within this of the peak
    "window": 20,             # half size of the centroid window, px (12 missed 13/53 close spots)
    "border_fraction": 0.5,   # core reaching the window edge -> too large (glare)
    "paper_margin_mm": 4.0,
    "gate_deg": 3.0,
    "max_correction_deg": 4.0,
}

MAX_TILT_DEG = 65.0
MAX_REPROJECTION_ERROR_PX = 3.0
# The pose residual is ~1% of the frame's diagonal in pixels at any range (109 captures),
# so a fixed 3 px gate rejects close or large views: field 2026-10-22, after DRIVE 20 60 the
# target was found every frame but rejected (pose reprojection error) until restarted.
# Gate = max(MAX_REPROJECTION_ERROR_PX, this fraction x diagonal). 0 = fixed gate only.
# 1.5% still rejected the parked end position of DRIVE 20 60 every frame (5.0-8.1 px against
# 4.1 px, diagonal ~273 px, frame found 21/21): 3% keeps tracking there. Outliers are still
# caught by the gimbal's 5 deg track gate. Lower it again once that view is understood.
# 2026-10-24 laps: the residual grows with the viewing angle (the pose is a rigidified
# homography). Parked mid side edge, target found 23/23 but 5.5-6.5 px against 4.6 px (3%,
# diagonal ~153 px) -> 30-50% valid frames on both side edges. 5% passes that view.
MAX_REPROJECTION_ERROR_FRACTION = 0.05
MIN_DEPTH_MM = 250.0
MAX_DEPTH_MM = 4000.0  # Camera z depth, not distance perpendicular to the target.
MAX_COMMAND_DEG = 60.0

# The 500 Hz controller owns direction, gains and dead zone. Do not apply them here.
# No fabricated intrinsics/laser lookup table: default operation sends valid=0
# until calibration.json contains both camera.calibrated and laser.calibrated.
