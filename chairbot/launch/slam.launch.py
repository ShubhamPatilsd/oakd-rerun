# On Pi:
# source /opt/ros/jazzy/setup.bash
# source ~/oakd-rerun/install/setup.bash
# ros2 launch chairbot slam.launch.py rerun_host:=<MAC_IP>
#
# On Mac:
# pip install rerun-sdk
# rerun

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rerun_host_arg = DeclareLaunchArgument(
        "rerun_host",
        default_value="192.168.1.100",
        description="IP address of the Mac running the Rerun viewer",
    )

    depthai_node = Node(
        package="depthai_ros_driver",
        executable="camera_node",
        name="oak_d_lite",
        output="screen",
        parameters=[
            {
                "camera_model": "OAK-D-LITE",
                "rgb_resolution": "1080p",
                "depth_resolution": "400p",
                "rgb_fps": 15.0,
                "depth_fps": 15.0,
            }
        ],
    )

    rtabmap_odom_node = Node(
        package="rtabmap_odom",
        executable="rgbd_odometry",
        name="rtabmap_odom",
        output="screen",
        parameters=[
            {
                "frame_id": "oak_d_lite_rgb_camera_optical_frame",
                "approx_sync": True,
                "queue_size": 10,
            }
        ],
        remappings=[
            ("/rgb/image", "/oak_d_lite/rgb/image_raw"),
            ("/rgb/camera_info", "/oak_d_lite/rgb/camera_info"),
            ("/depth/image", "/oak_d_lite/stereo/image_raw"),
            ("/depth/camera_info", "/oak_d_lite/stereo/camera_info"),
        ],
    )

    rtabmap_node = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        output="screen",
        parameters=[
            {
                "frame_id": "oak_d_lite_rgb_camera_optical_frame",
                "subscribe_depth": True,
                "subscribe_rgb": True,
                "queue_size": 10,
                "approx_sync": True,
                "Mem/STMSize": "30",
                "Vis/MaxFeatures": "500",
                "RGBD/AngularUpdate": "0.01",
                "RGBD/LinearUpdate": "0.01",
                "Rtabmap/DetectionRate": "1",
            }
        ],
        remappings=[
            ("/rgb/image", "/oak_d_lite/rgb/image_raw"),
            ("/rgb/camera_info", "/oak_d_lite/rgb/camera_info"),
            ("/depth/image", "/oak_d_lite/stereo/image_raw"),
            ("/depth/camera_info", "/oak_d_lite/stereo/camera_info"),
        ],
    )

    rerun_bridge_node = Node(
        package="chairbot",
        executable="rerun_bridge",
        name="rerun_bridge",
        output="screen",
        parameters=[
            {"rerun_host": LaunchConfiguration("rerun_host")}
        ],
    )

    return LaunchDescription(
        [
            rerun_host_arg,
            depthai_node,
            rtabmap_odom_node,
            rtabmap_node,
            rerun_bridge_node,
        ]
    )
