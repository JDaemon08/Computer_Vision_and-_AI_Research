import open3d as o3d
import numpy as np
import threading
import time
from config import (
    CAM_FX, CAM_FY, CAM_PPX, CAM_PPY,
    CAMERA_WIDTH, CAMERA_HEIGHT
)


SLAM_FITNESS_THRESHOLD = 0.3
SLAM_DEPTH_SCALE       = 1000.0
SLAM_DEPTH_MAX         = 3.0
SLAM_DOWNSAMPLE        = 2

class SLAMTracker:
    """
    RGB-D Odometry based camera pose tracker
    Uses Open3D's compute_rgbd_odometry to estimate
    camera movement between consecurive frames.
    Falls back to IMU rotation when tracking is lost
    """

    def __init__(self):
        self._lock      = threading.Lock()
        self._pose      = np.eye(4)
        self._prev_rgbd = None
        self._tracking  = False
        self._imu       = None

        self._intrinsics = o3d.camera.PinholeCameraIntrinsic(
            width  = CAMERA_WIDTH,
            height = CAMERA_HEIGHT,
            fx     = CAM_FX,
            fy     = CAM_FY,
            cx     = CAM_PPX,
            cy     = CAM_PPY
        )

        self._method  = o3d.pipelines.odometry.RGBDOdometryJacobianFromHybridTerm()
        self._options  = o3d.pipelines.odometry.OdometryOption(
            depth_max = SLAM_DEPTH_MAX
        )

    def attach_imu(self, imu_tracker):
        """Attach IMU tracking when SLAM loses tracking"""
        self._imu = imu_tracker
        print("IMU fallback attached to SLAM")

    def _make_rgbd(self, color_image, depth_image):
        """Convert numpy color + depth arrays into an
        Open3D RGBDimage
        """

        h = CAMERA_HEIGHT // SLAM_DOWNSAMPLE
        w = CAMERA_WIDTH // SLAM_DOWNSAMPLE

        import cv2
        color_small = cv2.resize(color_image, (w, h))
        depth_small = cv2.resize(depth_image, (w, h))
        interpolation=cv2.INTER_NEAREST

        color_rgb = cv2.cvtColor(color_small, cv2.COLOR_BGR2RGB)

        o3d_color = o3d.geometry.Image(color_rgb.astype(np.uint8))
        o3d_depth = o3d.geometry.Image(depth_small.astype(np.uint16))

        return o3d.geometry.RGBDImage.create_from_color_and_depth(
            o3d_color,
            o3d_depth,
            depth_scale = SLAM_DEPTH_SCALE,
            depth_trunc = SLAM_DEPTH_MAX,
            convert_rgb_to_intensity = False
        )

    def process_frame(self, color_image, depth_image):

        if color_image is None or depth_image is None:
            return

        rgbd = self._make_rgbd(color_image, depth_image)

        if self._prev_rgbd is None:
            self._prev_rgbd = rgbd
            self._tracking = True
            return

        success, T, info = o3d.pipelines.odometry.compute_rgbd_odometry(
            self._prev_rgbd,
            rgbd,
            self._intrinsics,
            np.eye(4),
            self._method,
            self._options
        )

        fitness = info[0, 0] if info is not None else 0.0

        if success and fitness >= SLAM_FITNESS_THRESHOLD:
            with self._lock:
                self._pose      = self._pose @ T
                self._tracking  = True
            self._prev_rgbd = rgbd

        else:
            with self._lock:
                self._tracking = False

            if fitness < SLAM_FITNESS_THRESHOLD:
                print(f"[SLAM] Low fitness ({fitness:.3f}) - using IMU fallback")
            else:
                print("[SLAM] Odometry failed - Using IMU fallback")

    def get_pose_matrix(self):
        "Returns 4x4 pose matrix"

        with self._lock:
            tracking = self._tracking
            pose     = self._pose.copy()

        if tracking:
            return pose

        if self._imu is not None:
            R = self._imu.get_rotation_matrix()
            t = pose[:3,3]
            fallback = np.eye[4]
            fallback[:3, :3] = R
            fallback[:3, 3] = t
            return fallback

        return pose

    def get_rotation_matrix(self):
        return self.get_pose_matrix()[:3,:3]

    def is_tracking(self):
        with self._lock:
            return self._tracking

    def reset(self):
        with self._lock:
            self._pose      = np.eye[4]
            self._tracking  = False
        self._prev_rgbd = None
        print(f"[SLAM] Pose reset to origin")

    def stop(self):
        print("SLAM tracker stopped")


