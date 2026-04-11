"""
main.py — Starts the camera feed and Rerun bridge as concurrent threads.
"""

import logging
import threading

import rtabmap_python as rtabmap_py

from camera_feed import run_camera
from rerun_bridge import run_bridge

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
)
log = logging.getLogger("main")


def main() -> None:
    rt = rtabmap_py.Rtabmap()
    rt.init("rtabmap.db")  # persists the map across runs; delete to start fresh

    # Shared state between camera thread (writer) and bridge thread (reader)
    shared: dict = {
        "pose": None,
        "rgb": None,
        "depth": None,
        "map_pts": None,
        "map_colors": None,
        "frame": 0,
    }
    lock = threading.Lock()
    stop = threading.Event()

    camera_thread = threading.Thread(
        target=run_camera,
        args=(rt, shared, lock, stop),
        name="camera",
        daemon=True,
    )
    bridge_thread = threading.Thread(
        target=run_bridge,
        args=(shared, lock, stop),
        name="rerun-bridge",
        daemon=True,
    )

    camera_thread.start()
    bridge_thread.start()
    log.info("Running — press Ctrl+C to stop.")

    try:
        # Block main thread; daemon threads run in background
        camera_thread.join()
        bridge_thread.join()
    except KeyboardInterrupt:
        log.info("Interrupt received, shutting down...")
        stop.set()
        camera_thread.join(timeout=5)
        bridge_thread.join(timeout=5)
        rt.close()
        log.info("Done.")


if __name__ == "__main__":
    main()
