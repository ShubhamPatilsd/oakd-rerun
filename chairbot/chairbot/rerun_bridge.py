"""
rerun_bridge.py — ROS 2 node that subscribes to rtabmap outputs and streams
everything to the Rerun viewer running on a Mac.

Topics consumed:
  /camera/color/image_raw    (sensor_msgs/Image)   — synced with depth
  /camera/depth/image_raw    (sensor_msgs/Image)   — synced with RGB
  /rtabmap/odom              (nav_msgs/Odometry)   — camera pose
  /rtabmap/cloud_map         (sensor_msgs/PointCloud2) — global map

Rerun entities:
  world/                    ViewCoordinates (static)
  world/camera              Transform3D    (from odometry)
  world/camera/rgb          Image
  world/camera/depth        DepthImage
  world/map                 Points3D
  world/trajectory          LineStrips3D
"""

import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

import rerun as rr


def _quat_to_rot3x3(x: float, y: float, z: float, w: float) -> np.ndarray:
    """Convert unit quaternion (x, y, z, w) to a 3x3 rotation matrix."""
    x2, y2, z2 = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array(
        [
            [1 - 2 * (y2 + z2),  2 * (xy - wz),      2 * (xz + wy)],
            [2 * (xy + wz),      1 - 2 * (x2 + z2),  2 * (yz - wx)],
            [2 * (xz - wy),      2 * (yz + wx),       1 - 2 * (x2 + y2)],
        ],
        dtype=np.float64,
    )


class RerunBridge(Node):
    def __init__(self) -> None:
        super().__init__("rerun_bridge")

        self.declare_parameter("rerun_host", "192.168.1.100")
        rerun_host = self.get_parameter("rerun_host").get_parameter_value().string_value

        rr.init("chairbot_slam")
        rr.connect_grpc(f"rerun+http://{rerun_host}:9876/proxy")
        self.get_logger().info(f"Rerun connected to {rerun_host}:9876")

        rr.log("world/", rr.ViewCoordinates.RIGHT_HAND_Y_DOWN, static=True)

        self._bridge = CvBridge()
        self.frame_count = 0
        self.trajectory: list[list[float]] = []

        self._last_image_log: float = 0.0
        self._last_map_log: float = 0.0
        self._image_min_interval = 1.0 / 10.0   # 10 fps max
        self._map_min_interval = 1.0             # 1 hz max

        # Synchronised RGB + depth
        rgb_sub = Subscriber(self, Image, "/camera/color/image_raw")
        depth_sub = Subscriber(self, Image, "/camera/depth/image_raw")
        self._sync = ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub], queue_size=10, slop=0.1
        )
        self._sync.registerCallback(self.image_callback)

        self.create_subscription(Odometry, "/rtabmap/odom", self.odom_callback, 10)
        self.create_subscription(
            PointCloud2, "/rtabmap/cloud_map", self.pointcloud_callback, 10
        )

    # ------------------------------------------------------------------
    def image_callback(self, rgb_msg: Image, depth_msg: Image) -> None:
        now = time.monotonic()
        if now - self._last_image_log < self._image_min_interval:
            return
        self._last_image_log = now

        bgr = self._bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

        depth = self._bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")

        rr.set_time_sequence("frame", self.frame_count)
        rr.log("world/camera/rgb", rr.Image(rgb))
        rr.log("world/camera/depth", rr.DepthImage(depth, meter=1000.0))

        self.frame_count += 1

    # ------------------------------------------------------------------
    def odom_callback(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        translation = [p.x, p.y, p.z]
        R = _quat_to_rot3x3(q.x, q.y, q.z, q.w)

        rr.set_time_sequence("frame", self.frame_count)
        rr.log(
            "world/camera",
            rr.Transform3D(translation=translation, mat3x3=R),
        )

        self.trajectory.append(translation)
        if len(self.trajectory) > 1:
            rr.log(
                "world/trajectory",
                rr.LineStrips3D([self.trajectory], colors=[[0, 255, 0]]),
            )

    # ------------------------------------------------------------------
    def pointcloud_callback(self, msg: PointCloud2) -> None:
        now = time.monotonic()
        if now - self._last_map_log < self._map_min_interval:
            return
        self._last_map_log = now

        field_names = [f.name for f in msg.fields]
        has_rgb = "rgb" in field_names or "rgba" in field_names

        cloud = list(
            pc2.read_points(msg, field_names=("x", "y", "z") + (("rgb",) if has_rgb else ()), skip_nans=True)
        )
        if not cloud:
            return

        points = np.array([[p[0], p[1], p[2]] for p in cloud], dtype=np.float32)

        if has_rgb:
            # RGB is packed as a float32 whose bytes are 0xBBGGRR00
            raw = np.array([p[3] for p in cloud], dtype=np.float32)
            packed = raw.view(np.uint32)
            r = ((packed >> 16) & 0xFF).astype(np.uint8)
            g = ((packed >> 8) & 0xFF).astype(np.uint8)
            b = (packed & 0xFF).astype(np.uint8)
            colors = np.stack([r, g, b], axis=-1)
            rr.log("world/map", rr.Points3D(points, colors=colors, radii=0.02))
        else:
            rr.log("world/map", rr.Points3D(points, radii=0.02))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RerunBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
