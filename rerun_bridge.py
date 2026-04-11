"""
rerun_bridge.py — Reads shared RTABMap state and streams it to Rerun on the Mac.
"""

import logging
import threading
import time

import numpy as np
import rerun as rr

log = logging.getLogger("rerun_bridge")

LAPTOP_IP = "192.168.1.100"  # <-- set your Mac's IP here
RERUN_PORT = 9876
POLL_INTERVAL = 0.05  # seconds between Rerun log calls (~20fps visual refresh)


def run_bridge(
    shared: dict,
    lock: threading.Lock,
    stop: threading.Event,
) -> None:
    """Reads from shared state dict and streams everything to Rerun."""
    rr.init("rtabmap_oak")
    rr.connect(f"{LAPTOP_IP}:{RERUN_PORT}")
    log.info(f"Rerun connected to {LAPTOP_IP}:{RERUN_PORT}")

    trajectory: list[np.ndarray] = []
    last_frame = -1

    while not stop.is_set():
        # Snapshot shared state atomically — don't hold the lock during Rerun calls
        with lock:
            frame_idx = shared.get("frame", 0)
            pose = shared.get("pose")
            rgb = shared.get("rgb")
            depth = shared.get("depth")
            pts = shared.get("map_pts")
            colors = shared.get("map_colors")

        # Skip if no data yet or same frame as last iteration
        if rgb is None or frame_idx == last_frame:
            time.sleep(POLL_INTERVAL)
            continue

        last_frame = frame_idx
        rr.set_time_sequence("frame", frame_idx)

        # Camera pose
        if pose is not None:
            rr.log(
                "world/camera",
                rr.Transform3D(
                    translation=pose[:3, 3],
                    mat3x3=pose[:3, :3],
                ),
            )
            trajectory.append(pose[:3, 3].copy())

        # RGB image (BGR from OpenCV)
        rr.log("world/camera/rgb", rr.Image(rgb, color_model="BGR"))

        # Depth image — OAK-D Lite depth is in millimetres
        if depth is not None:
            rr.log("world/camera/depth", rr.DepthImage(depth, meter=1000.0))

        # Map point cloud
        if pts is not None and len(pts) > 0:
            if colors is not None and len(colors) == len(pts):
                # colors may be float [0,1] or uint8 [0,255] depending on rtabmap build
                if colors.dtype != np.uint8:
                    colors = (np.clip(colors, 0.0, 1.0) * 255).astype(np.uint8)
                rr.log("world/map", rr.Points3D(pts, colors=colors, radii=0.02))
            else:
                rr.log("world/map", rr.Points3D(pts, radii=0.02))

        # Trajectory line strip
        if len(trajectory) > 1:
            rr.log(
                "world/trajectory",
                rr.LineStrips3D([trajectory], colors=[[0, 255, 0]]),
            )

        time.sleep(POLL_INTERVAL)
