import open3d as o3d
import numpy as np
import threading
from collections import deque
from config import (
    CAM_FX, CAM_FY, CAM_PPX, CAM_PPY,
    MAP_MAX_POINTS,
    MAP_POINT_SIZE,
    MAP_UPDATE_EVERY_N
)

CLASS_COLORS = {
    "person":       [1.0, 0.0, 0.0],   # red
    "bicycle":      [0.0, 1.0, 0.0],   # green
    "car":          [0.0, 0.0, 1.0],   # blue
    "motorcycle":   [1.0, 0.5, 0.0],   # orange
    "bus":          [0.5, 0.0, 0.5],   # purple
    "truck":        [0.0, 0.5, 0.5],   # teal
    "cup":          [1.0, 1.0, 0.0],   # yellow
    "bottle":       [0.0, 1.0, 1.0],   # cyan
    "chair":        [1.0, 0.0, 1.0],   # magenta
    "laptop":       [0.5, 1.0, 0.0],   # lime
    "phone":        [1.0, 0.0, 0.5],   # pink
    "cell phone":   [1.0, 0.0, 0.5],   # pink
    "book":         [0.5, 0.5, 0.0],   # olive
    "keyboard":     [0.0, 0.5, 1.0],   # sky blue
    "mouse":        [0.8, 0.4, 0.2],   # brown
}
DEFAULT_COLOR = [0.7, 0.7, 0.7]

def pixel_to_3d(cx, cy, depth_cm):

    """
    Converts a pixel (cx, cy) and depth in cm
    to real world (X, Y, Z) coordinates in meters
    """

    depth_m = depth_cm / 100.0
    X = -(cx - CAM_FX) * depth_m /CAM_FX
    Y = -(cy - CAM_FY) * depth_m /CAM_FX
    Z = depth_m

    return X, Y, Z

class PointMapper:

    """
    Accumulates 3D detections over time and renders them in a live 3D visualizer.
    OBS: Runs visualizer on a separate thread as to not interfere with camera pipeline.
    """

    def __init__(self):
        self.points = deque(maxlen=MAP_MAX_POINTS)
        self.colors = deque(maxlen=MAP_MAX_POINTS)

        self.frame_count = 0
        self._lock       = threading.Lock()

        self.pcd = o3d.geometry.PointCloud()
        self.vis = o3d.visualization.Visualizer()

        self._thread = threading.Thread(target=self._run_visualizer, daemon=True)
        self._thread.start()

    def _run_visualizer(self):
        """
        Initializes and runs Open3D window

        """
        self.vis.create_window(window_name= "3D point Map", width=960, height=540)
        self.vis.add_geometry(self.pcd)
        render_opt = self.vis.get_render_option()
        render_opt.point_size   = MAP_POINT_SIZE
        render_opt.background_color = np.array([0.1,0.1,0.1])

        coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=0.5, origin=[0,0,0]
        )
        self.vis.add_geometry(coord_frame)

        while True:
            with self._lock:
                if len(self.points) > 0:
                    self.pcd.points = o3d.utility.Vector3dVector(
                        np.array(self.points)
                    )
                    self.pcd.colors = o3d.utility.Vector3dVector(
                        np.array(self.colors)
                    )
                    self.vis.update_geometry(self.pcd)

            if not self.vis.poll_events():
                break
            self.vis.update_renderer()

    def update(self, detections):
        """
        Receives list of Detection objects from main.py, converts each into a 3D point and adds it to the map.
        Processes only N frames for performance
        """

        self.frame_count += 1
        if self.frame_count % MAP_UPDATE_EVERY_N != 0:
            return
        
        with self._lock:
            for det in detections:
                if det.distance_cm <= 0.0:
                    continue

                X, Y, Z = pixel_to_3d(det.cx, det.cy, det.distance_cm)

                if Z<= 0.0 or Z > 10.0:
                    continue

                color = CLASS_COLORS.get(det.label, DEFAULT_COLOR)
                self.points.append([X, Y, Z])
                self.colors.append(color)

    def stop(self):
        self.vis.destroy_window()
