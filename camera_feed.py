"""
camera_feed.py — Captures RGB-D from OAK-D Lite and feeds frames into RTABMap.

RTABMap API surface used (verify against your rtabmap_python build):
  rt.process(rgb, depth, fx, fy, cx, cy, stamp)  -> None
  rt.getPose()                                    -> np.ndarray (4x4) or None
  rt.getMap()                                     -> (pts: Nx3, colors: Nx3 uint8) or (None, None)
"""

import logging
import time
import threading

import cv2
import depthai as dai
import numpy as np
import rtabmap_python as rtabmap_py

log = logging.getLogger("camera_feed")

TARGET_FPS = 10
FRAME_INTERVAL = 1.0 / TARGET_FPS
W, H = 640, 480
RECONNECT_DELAY = 3.0


def _build_pipeline() -> dai.Pipeline:
    pipeline = dai.Pipeline()

    # RGB — downsample from 1080p to 640x480 via preview
    cam_rgb = pipeline.create(dai.node.ColorCamera)
    cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    cam_rgb.setPreviewSize(W, H)
    cam_rgb.setInterleaved(False)
    cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

    # Stereo pair — OAK-D Lite mono cameras are OV7251 at 480p
    mono_left = pipeline.create(dai.node.MonoCamera)
    mono_right = pipeline.create(dai.node.MonoCamera)
    mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_480_P)
    mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_480_P)
    mono_left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
    mono_right.setBoardSocket(dai.CameraBoardSocket.CAM_C)

    # Stereo depth — align to RGB lens, output at 640x480
    stereo = pipeline.create(dai.node.StereoDepth)
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
    stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
    stereo.setOutputSize(W, H)
    mono_left.out.link(stereo.left)
    mono_right.out.link(stereo.right)

    # Output queues — maxSize=1 so we always get the freshest frame
    xout_rgb = pipeline.create(dai.node.XLinkOut)
    xout_depth = pipeline.create(dai.node.XLinkOut)
    xout_rgb.setStreamName("rgb")
    xout_depth.setStreamName("depth")
    cam_rgb.preview.link(xout_rgb.input)
    stereo.depth.link(xout_depth.input)

    return pipeline


def run_camera(
    rt: rtabmap_py.Rtabmap,
    shared: dict,
    lock: threading.Lock,
    stop: threading.Event,
) -> None:
    """Main camera loop. Runs until stop is set. Reconnects on disconnect."""
    pipeline = _build_pipeline()

    while not stop.is_set():
        try:
            log.info("Connecting to OAK-D Lite...")
            with dai.Device(pipeline) as device:
                # Pull factory calibration from the device, projected to W x H
                calib = device.readCalibration()
                M = calib.getCameraIntrinsics(dai.CameraBoardSocket.CAM_A, W, H)
                fx, fy = M[0][0], M[1][1]
                cx, cy = M[0][2], M[1][2]
                log.info(f"Intrinsics — fx={fx:.2f} fy={fy:.2f} cx={cx:.2f} cy={cy:.2f}")

                rgb_q = device.getOutputQueue("rgb", maxSize=1, blocking=False)
                depth_q = device.getOutputQueue("depth", maxSize=1, blocking=False)

                frame_idx = 0
                next_tick = time.monotonic()

                while not stop.is_set():
                    # Rate-limit to TARGET_FPS; skip processing if we're behind
                    now = time.monotonic()
                    sleep_for = next_tick - now
                    if sleep_for > 0:
                        time.sleep(sleep_for)
                    next_tick = time.monotonic() + FRAME_INTERVAL

                    rgb_msg = rgb_q.tryGet()
                    depth_msg = depth_q.tryGet()
                    if rgb_msg is None or depth_msg is None:
                        continue

                    rgb: np.ndarray = rgb_msg.getCvFrame()    # (H, W, 3) BGR uint8
                    depth: np.ndarray = depth_msg.getFrame()  # (H, W) uint16, mm
                    stamp: float = time.time()

                    # Hand off to RTABMap — this call may block up to ~100ms
                    rt.process(rgb, depth, fx, fy, cx, cy, stamp)

                    # Read results immediately after process() while state is fresh
                    pose = rt.getPose()          # 4x4 float64 or None if lost
                    pts, colors = rt.getMap()    # (Nx3, Nx3 uint8) or (None, None)

                    with lock:
                        shared["pose"] = pose
                        shared["rgb"] = rgb.copy()
                        shared["depth"] = depth.copy()
                        shared["map_pts"] = pts
                        shared["map_colors"] = colors
                        shared["frame"] = frame_idx

                    frame_idx += 1

        except dai.XLinkError:
            log.warning(f"Camera disconnected. Retrying in {RECONNECT_DELAY}s...")
            time.sleep(RECONNECT_DELAY)
        except Exception as exc:
            log.error(f"Camera error: {exc!r}. Retrying in {RECONNECT_DELAY}s...")
            time.sleep(RECONNECT_DELAY)
