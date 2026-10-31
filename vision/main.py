"""Run with CanMV IDE on Yahboom K230 / GC2093, without a physical LCD."""
import sys
import os
import time
import gc
import json



APP_DIR = "/sdcard/k230_vision"


def locate_app_directory():
    # IDE execution may omit __file__; files must still exist on the BOARD.
    candidates = []
    script = globals().get("__file__", "").replace("\\", "/")
    if "/" in script:
        candidates.append(script.rsplit("/", 1)[0])
    candidates.append(APP_DIR)
    candidates.append(os.getcwd())
    candidates.append("/sdcard")
    modules = ("config.py", "protocol.py", "runtime.py", "detector.py", "geometry.py", "processor.py", "spot.py")
    for directory in candidates:
        try:
            os.stat(directory + "/config.py")
        except OSError:
            continue
        missing = []
        for name in modules:
            try:
                os.stat(directory + "/" + name)
            except OSError:
                missing.append(name)
        if missing:
            raise ImportError("Incomplete K230 files in " + directory + ": " + ", ".join(missing))
        return directory
    raise ImportError("Copy ALL vision/*.py files onto the K230 SD card in /sdcard/k230_vision/. "
                      "Running main.py from the PC alone does not provide config.py on the board.")


APP_DIR = locate_app_directory()
if APP_DIR in sys.path:
    sys.path.remove(APP_DIR)
sys.path.insert(0, APP_DIR)

import config as cfg
from protocol import VisionSender, TaskReceiver
from runtime import publish_result

# Relative data paths follow the deployed scripts, including SD-root deployments.
CALIBRATION_FILE = (cfg.CALIBRATION_FILE if cfg.CALIBRATION_FILE.startswith("/")
                    else APP_DIR + "/" + cfg.CALIBRATION_FILE)
CAPTURE_DIRECTORY = (cfg.CAPTURE_DIRECTORY if cfg.CAPTURE_DIRECTORY.startswith("/")
                     else APP_DIR + "/" + cfg.CAPTURE_DIRECTORY)


def apply_exposure(sensor, exposure_us, gain):
    """Fixed exposure/gain if configured (after run()), then report what is in use."""
    try:
        if exposure_us:
            sensor.exposure(exposure_us)
        if gain:
            sensor.again(gain)
    except Exception as exc:
        print("Setting exposure/gain failed:", exc)
    try:
        print("Exposure: auto={} time_us={} again={}".format(
            sensor.auto_exposure(), sensor.exposure(), sensor.again()))
    except Exception as exc:
        print("Exposure readback unavailable (firmware < v1.6?):", exc)


def load_calibration():
    try:
        with open(CALIBRATION_FILE, "r") as stream:
            return json.load(stream)
    except (OSError, ValueError) as exc:
        print("Calibration unavailable:", exc)
        return None


def make_directories(path):
    # MicroPython has no os.makedirs; create each level (e.g. captures/laser).
    current = ""
    for part in path.split("/"):
        if not part:
            current = current or "/"
            continue
        current = part if not current else (current.rstrip("/") + "/" + part)
        try:
            os.mkdir(current)
        except OSError:
            pass


def capture_path(now):
    # ticks_ms restarts after reboot; never overwrite a previous calibration image.
    index = 0
    while True:
        path = CAPTURE_DIRECTORY + "/frame_{}_{}.bmp".format(now, index)
        try:
            os.stat(path)
        except OSError as exc:
            if exc.args and exc.args[0] == 2:  # ENOENT only
                return path
            raise
        index += 1


def show_preview(display, img, result):
    # Draw only on a separate RGB888 image; detection and saved raw images stay clean.
    preview = img.to_rgb888()
    observation = result.get("observation")
    if observation:
        from geometry import homography, transform
        corners = observation["corners"]
        color = (0, 255, 0) if result["valid"] else (255, 220, 0)
        for i in range(4):
            a, b = corners[i], corners[(i + 1) % 4]
            preview.draw_line(int(a[0]), int(a[1]), int(b[0]), int(b[1]), color=color, thickness=2)
        try:
            mapping = homography(((0, 0), (cfg.TARGET_WIDTH_MM, 0),
                                  (cfg.TARGET_WIDTH_MM, cfg.TARGET_HEIGHT_MM),
                                  (0, cfg.TARGET_HEIGHT_MM)), corners)
            x, y = transform(mapping, cfg.TARGET_CENTER_MM)
            x, y = int(x), int(y)
            preview.draw_line(x-6, y, x+6, y, color=(0, 255, 255), thickness=2)
            preview.draw_line(x, y-6, x, y+6, color=(0, 255, 255), thickness=2)
        except ValueError:
            pass
    display.show_image(preview)


def main():
    from machine import UART, FPIOA
    from media.sensor import Sensor
    from media.media import MediaManager

    sensor = uart = sender = receiver = display = frame_log = None
    media_initialized = False
    try:
        fpioa = FPIOA()
        fpioa.set_function(cfg.UART_TX_PIN, FPIOA.UART1_TXD)
        fpioa.set_function(cfg.UART_RX_PIN, FPIOA.UART1_RXD)
        try:        # timeout 0: reading the gimbal's $K must never hold up a frame
            uart = UART(UART.UART1, baudrate=cfg.UART_BAUD, bits=UART.EIGHTBITS,
                        parity=UART.PARITY_NONE, stop=UART.STOPBITS_ONE, timeout=0)
        except TypeError:   # firmware without the argument
            uart = UART(UART.UART1, baudrate=cfg.UART_BAUD, bits=UART.EIGHTBITS,
                        parity=UART.PARITY_NONE, stop=UART.STOPBITS_ONE)
        sender = VisionSender(uart)
        sender.send(False)
        receiver = TaskReceiver(uart)        # $K circle task from the gimbal, same UART
        # Import cv2 after UART setup: a missing firmware module also clears vision.
        from detector import FrameDetector
        from processor import VisionProcessor
        detector = FrameDetector(cfg.WIDTH, cfg.HEIGHT, cfg.DETECTION,
                                 (cfg.TARGET_WIDTH_MM, cfg.TARGET_HEIGHT_MM), cfg.TAPE_WIDTH_MM)
        processor = VisionProcessor(cfg, detector, load_calibration())
        print("Firmware:", os.uname())
        print("Camera: GC2093", "GRAYSCALE" if cfg.SENSOR_GRAYSCALE else "RGB565",
              cfg.WIDTH, cfg.HEIGHT, "LCD: none")
        print("Debug: preview={} interval_ms={} capture_ms={} threshold={}".format(
            cfg.IDE_PREVIEW, cfg.PREVIEW_INTERVAL_MS, cfg.CAPTURE_INTERVAL_MS,
            cfg.DETECTION["threshold"]))
        if processor.calibration_error:
            print("CONTROL DISABLED:", processor.calibration_error)
            print("Collect camera calibration and measured laser spots before closed-loop use.")

        sensor = Sensor()
        sensor.reset()
        sensor.set_framesize(width=cfg.WIDTH, height=cfg.HEIGHT)
        sensor.set_pixformat(Sensor.GRAYSCALE if cfg.SENSOR_GRAYSCALE else Sensor.RGB565)
        sensor.set_hmirror(cfg.HMIRROR)
        sensor.set_vflip(cfg.VFLIP)
        manual_exposure = getattr(cfg, "EXPOSURE_US", None)
        if manual_exposure:
            try:
                sensor.auto_exposure(False)          # must happen before run()
            except Exception as exc:
                print("Manual exposure unavailable (firmware < v1.6?):", exc)
                manual_exposure = None
        if cfg.USE_VIRTUAL_DISPLAY or cfg.IDE_PREVIEW:
            from media.display import Display
            display = Display
            preview_fps = max(1, min(60, 1000 // max(1, cfg.PREVIEW_INTERVAL_MS)))
            display.init(Display.VIRT, width=cfg.WIDTH, height=cfg.HEIGHT,
                         fps=preview_fps, to_ide=True)
        MediaManager.init()
        media_initialized = True
        sensor.run()
        time.sleep_ms(cfg.STARTUP_SETTLE_MS)
        apply_exposure(sensor, manual_exposure, getattr(cfg, "ANALOG_GAIN", None))

        frame_log = None
        if getattr(cfg, "FRAME_LOG", False):
            from framelog import FrameLog
            make_directories(CAPTURE_DIRECTORY)
            frame_log = FrameLog("{}/framelog_{}.csv".format(CAPTURE_DIRECTORY, time.ticks_ms()),
                                 getattr(cfg, "FRAME_LOG_FLUSH_MS", 10000))
            print("Frame log:", frame_log.path)
        last_log = last_capture = time.ticks_ms()
        last_preview = None
        frames = since_gc = 0
        found_frames = preview_total_ms = save_total_ms = process_max_ms = poll_max_ms = 0
        while True:
            os.exitpoint()
            started = time.ticks_ms()
            img = sensor.snapshot()
            # Keep the owner alive while cv2 accesses the shared grayscale buffer.
            gray_img = img if cfg.SENSOR_GRAYSCALE else img.to_grayscale()
            gray = gray_img.to_numpy_ref()
            poll_started = time.ticks_ms()
            task = receiver.poll()           # never blocks; newest $K since the last frame
            poll_max_ms = max(poll_max_ms, time.ticks_diff(time.ticks_ms(), poll_started))
            if task:
                processor.set_task(task[0], task[1])
            result = processor.process(gray)
            processing_reason = result["reason"]
            spot_status = result.get("spot", "spot_off")
            correction = result.get("correction_deg", (0.0, 0.0))
            if result.get("observation") is not None:
                found_frames += 1
            elapsed = time.ticks_diff(time.ticks_ms(), started)
            process_max_ms = max(process_max_ms, elapsed)
            if cfg.CAPTURE_INTERVAL_MS:
                result["valid"] = False
                result["reason"] = "calibration_capture"
            result = publish_result(sender, result, elapsed, cfg.MAX_RESULT_AGE_MS)
            frames += 1
            now = time.ticks_ms()
            if frame_log:
                frame_log.add(started, elapsed, result, detector)
                if frame_log.due(now):
                    frame_log.flush(now)
            if cfg.CAPTURE_INTERVAL_MS and time.ticks_diff(now, last_capture) >= cfg.CAPTURE_INTERVAL_MS:
                # Calibration capture pauses correction; SD I/O latency is unbounded.
                sender.send(False)
                save_started = time.ticks_ms()
                make_directories(CAPTURE_DIRECTORY)
                path = capture_path(now)
                img.save(path)
                print("Saved", path)
                last_capture = time.ticks_ms()
                save_total_ms += time.ticks_diff(last_capture, save_started)
            if cfg.IDE_PREVIEW and (last_preview is None or
                    time.ticks_diff(now, last_preview) >= cfg.PREVIEW_INTERVAL_MS):
                preview_started = time.ticks_ms()
                show_preview(display, img, result)
                last_preview = time.ticks_ms()
                preview_total_ms += time.ticks_diff(last_preview, preview_started)
            now = time.ticks_ms()
            if time.ticks_diff(now, last_log) >= cfg.LOG_INTERVAL_MS:
                seconds = max(1, time.ticks_diff(now, last_log)) / 1000.0
                print("fps={:.1f} ms={} valid={} reason={} yaw={:.3f} pitch={:.3f} tx_fail={} "
                      "det={} seen={}/{} source={} max_ms={} preview_ms={} save_ms={} thr={:.0f} {} corr={:+.2f}/{:+.2f} "
                      "task={}/{:.3f} rx={}/{} rx_ms={}".format(
                    frames/seconds, elapsed, int(result["valid"]), result["reason"],
                    result.get("yaw_deg", 0.0), result.get("pitch_deg", 0.0), sender.failures,
                    detector.last_status, found_frames, frames, processing_reason,
                    process_max_ms, preview_total_ms, save_total_ms, detector.threshold,
                    spot_status, correction[0], correction[1],
                    processor.task[0], processor.task[1], receiver.frames, receiver.errors, poll_max_ms))
                frames, last_log = 0, now
                found_frames = preview_total_ms = save_total_ms = process_max_ms = poll_max_ms = 0
            del gray, gray_img, img
            since_gc += 1
            if since_gc >= cfg.GC_INTERVAL_FRAMES:
                gc.collect()
                since_gc = 0
            if cfg.LOOP_YIELD_MS:
                time.sleep_ms(cfg.LOOP_YIELD_MS)
    except KeyboardInterrupt:
        print("Stopped by user")
        if frame_log:
            print("Frame log lines:", frame_log.lines_written + frame_log.flush())
    except Exception as exc:
        sys.print_exception(exc)
        print("Vision stopped. Verify board firmware/API and calibration.")
    finally:
        # One invalid result on exit. STM32 also has its own 300 ms timeout.
        if sender:
            sender.send(False)
        for resource, method in ((sensor, "stop"), (display, "deinit")):
            if resource:
                try:
                    getattr(resource, method)()
                except Exception as exc:
                    print("Cleanup:", exc)
        if media_initialized:
            try:
                MediaManager.deinit()
            except Exception as exc:
                print("Media cleanup:", exc)
        if uart:
            uart.deinit()


if __name__ == "__main__":
    os.exitpoint(os.EXITPOINT_ENABLE)
    main()
