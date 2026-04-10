import cv2
import depthai as dai
import numpy as np
import open3d as o3d
import rerun as rr


def build_pipeline() -> dai.Pipeline:
    pipeline = dai.Pipeline()

    cam_rgb = pipeline.create(dai.node.ColorCamera)
    cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    cam_rgb.setInterleaved(False)
    cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)
    cam_rgb.setPreviewSize(640, 400)

    mono_left = pipeline.create(dai.node.MonoCamera)
    mono_right = pipeline.create(dai.node.MonoCamera)
    mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    mono_left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
    mono_right.setBoardSocket(dai.CameraBoardSocket.CAM_C)

    stereo = pipeline.create(dai.node.StereoDepth)
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
    stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
    stereo.setOutputSize(640, 400)
    mono_left.out.link(stereo.left)
    mono_right.out.link(stereo.right)

    xout_rgb = pipeline.create(dai.node.XLinkOut)
    xout_depth = pipeline.create(dai.node.XLinkOut)
    xout_rgb.setStreamName("rgb")
    xout_depth.setStreamName("depth")
    cam_rgb.preview.link(xout_rgb.input)
    stereo.depth.link(xout_depth.input)

    return pipeline


def main():
    W, H = 640, 400
    fx = fy = 400.0
    cx, cy = W / 2, H / 2

    intrinsics = o3d.camera.PinholeCameraIntrinsic(W, H, fx, fy, cx, cy)

    rr.init("oak_slam", spawn=True)
    rr.log("world/camera", rr.Pinhole(focal_length=fx, width=W, height=H), static=True)

    pipeline = build_pipeline()

    pose = np.eye(4)
    prev_rgbd = None
    trajectory: list[np.ndarray] = []
    frame_idx = 0

    with dai.Device(pipeline) as device:
        rgb_queue = device.getOutputQueue("rgb", maxSize=4, blocking=False)
        depth_queue = device.getOutputQueue("depth", maxSize=4, blocking=False)

        print("Streaming — press Ctrl+C to stop.")
        while True:
            rgb_msg = rgb_queue.get()
            depth_msg = depth_queue.get()

            rgb_frame = rgb_msg.getCvFrame()          # (H, W, 3) BGR uint8
            depth_frame = depth_msg.getFrame()        # (H, W) uint16, mm

            color_o3d = o3d.geometry.Image(cv2.cvtColor(rgb_frame, cv2.COLOR_BGR2RGB))
            depth_o3d = o3d.geometry.Image(depth_frame.astype(np.uint16))
            rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
                color_o3d, depth_o3d,
                depth_scale=1000.0,
                depth_trunc=5.0,
                convert_rgb_to_intensity=False,
            )

            if prev_rgbd is not None:
                success, T, _ = o3d.pipelines.odometry.compute_rgbd_odometry(
                    prev_rgbd, rgbd,
                    intrinsics,
                    np.eye(4),
                    o3d.pipelines.odometry.RGBDOdometryJacobianFromHybridTerm(),
                )
                if success:
                    pose = pose @ T

            prev_rgbd = rgbd

            pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsics)
            pcd.transform(pose)
            pts = np.asarray(pcd.points)
            colors = (np.asarray(pcd.colors) * 255).astype(np.uint8)

            rr.set_time_sequence("frame", frame_idx)

            rr.log(
                "world/camera",
                rr.Transform3D(translation=pose[:3, 3], mat3x3=pose[:3, :3]),
            )
            rr.log("world/camera/rgb", rr.Image(rgb_frame, color_model="BGR"))
            rr.log("world/camera/depth", rr.DepthImage(depth_frame, meter=1000.0))

            if pts.shape[0] > 0:
                rr.log("world/map", rr.Points3D(pts, colors=colors, radii=0.005))

            trajectory.append(pose[:3, 3].copy())
            if len(trajectory) > 1:
                rr.log(
                    "world/trajectory",
                    rr.LineStrips3D([trajectory], colors=[[0, 255, 0]]),
                )

            frame_idx += 1


if __name__ == "__main__":
    main()
