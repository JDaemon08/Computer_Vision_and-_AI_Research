import open3d as o3d
import numpy as np
import threading
from collections import deque
from config import (
    CAM_FX, CAM_FY, CAM_PPX, CAM_PPY,
    MAP_MAX_POINTS,
    MAP_POINT_SIZE,
    MAP_UPDATE_EVERY_N,
    MAP_ENV_ENABLED,
    MAP_ENV_SUBSAMPLE,
    MAP_ENV_COLOR
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
    "cell phone":   [1.0, 0.0, 0.5],   # pink
    "book":         [0.5, 0.5, 0.0],   # olive
    "keyboard":     [0.0, 0.5, 1.0],   # sky blue
    "mouse":        [0.8, 0.4, 0.2],   # brown
}
DEFAULT_COLOR = [0.7, 0.7, 0.7]


def pixel_to_3d(cx, cy, depth_cm):
    """
    Converts a pixel (cx, cy) and depth in cm
    to real world (X, Y, Z) coordinates in meters.
    """
    depth_m = depth_cm / 100.0
    X = -((cx - CAM_PPX) * depth_m / CAM_FX)  
    Y = -((cy - CAM_PPY) * depth_m / CAM_FY)  
    Z = depth_m
    return X, Y, Z


# ─────────────────────────────────────────
# BASE CLASS
# ─────────────────────────────────────────
class _BaseMapper:
    """Shared logic for all mapper types."""

    def __init__(self, window_name, start_delay=0.0):
        self.window_name = window_name
        self.points      = deque(maxlen=MAP_MAX_POINTS)
        self.colors      = deque(maxlen=MAP_MAX_POINTS)
        self.frame_count = 0
        self._lock       = threading.Lock()
        self._escape     = False
        self.pcd         = o3d.geometry.PointCloud()
        self.vis         = o3d.visualization.VisualizerWithKeyCallback()
        self._start_delay = start_delay

        self._thread = threading.Thread(target=self._run_visualizer, daemon=True)
        self._thread.start()

    def _on_escape(self, vis):
        self._escape = True
        return False

    def _run_visualizer(self):
        if self._start_delay > 0:
            import time
            time.sleep(self._start_delay)

        self.vis.create_window(window_name=self.window_name, width=960, height=540)
        self.vis.register_key_callback(256, self._on_escape)

        render_opt = self.vis.get_render_option()
        render_opt.point_size       = MAP_POINT_SIZE
        render_opt.background_color = np.array([0.1, 0.1, 0.1])

        coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=0.3, origin=[0, 0, 0]
        )
        self.vis.add_geometry(coord_frame)

        pcd_added = False  

        while not self._escape:
            with self._lock:
                if len(self.points) > 0:
                    self.pcd.points = o3d.utility.Vector3dVector(np.array(self.points))
                    self.pcd.colors = o3d.utility.Vector3dVector(np.array(self.colors))
                    if not pcd_added:
                        self.vis.add_geometry(self.pcd)
                        pcd_added = True
                    else:
                        self.vis.update_geometry(self.pcd)

            if not self.vis.poll_events():
                break
            self.vis.update_renderer()

    def escaped(self):
        return self._escape

    def stop(self):
        try:
            self.vis.destroy_window()
        except Exception:
            pass

    def clear(self):
        with self._lock:
            self.points.clear()
            self.colors.clear()
        self.frame_count = 0
        self._escape     = False


class PointMapper(_BaseMapper):
    """Maps detected objects as colored 3D points."""

    def __init__(self, window_name="Detection Map"):
        super().__init__(window_name, start_delay=0.0)

    def update(self, detections):
        self.frame_count += 1
        if self.frame_count % MAP_UPDATE_EVERY_N != 0:
            return

        with self._lock:
            for det in detections:
                if det.distance_cm <= 0.0:
                    continue
                X, Y, Z = pixel_to_3d(det.cx, det.cy, det.distance_cm)
                if Z <= 0.0 or Z > 10.0:
                    continue
                self.points.append([X, Y, Z])
                self.colors.append(CLASS_COLORS.get(det.label, DEFAULT_COLOR))


class EnvironmentMapper(_BaseMapper):
    """Maps the environment as a dense point cloud, masking out detections."""

    def __init__(self, window_name="Environment Map", start_delay=0.5):
        self.start_delay = start_delay
        super().__init__(window_name, start_delay=0.0)

    def update(self, depth_image, detections):
        self.frame_count += 1
        if self.frame_count % MAP_UPDATE_EVERY_N != 0:
            return

        if depth_image is None:
            return

        step      = MAP_ENV_SUBSAMPLE
        depth_sub = depth_image[::step, ::step].astype(np.float32)

        mask = np.ones(depth_image.shape, dtype=bool)
        for det in detections:
            mask[det.y1:det.y2, det.x1:det.x2] = False
        mask_sub = mask[::step, ::step]

        valid_mask = (depth_sub > 0) & mask_sub
        rows, cols = np.where(valid_mask)
        rows_orig  = rows * step
        cols_orig  = cols * step

        if len(rows_orig) == 0:
            return

        depths  = depth_sub[rows, cols]
        depth_m = depths * 0.001

        X = -((cols_orig - CAM_PPX) * depth_m / CAM_FX)
        Y = -((rows_orig - CAM_PPY) * depth_m / CAM_FY)
        Z = depth_m

        range_mask = (Z > 0.0) & (Z <= 10.0)
        X = X[range_mask]
        Y = Y[range_mask]
        Z = Z[range_mask]

        if len(X) == 0:
            return

        new_points = np.stack([X, Y, Z], axis=1)
        new_colors = np.tile(MAP_ENV_COLOR, (len(X), 1))

        with self._lock:
            self.points.extend(new_points.tolist())
            self.colors.extend(new_colors.tolist())


class CombinedMapper:
    """Renders both detection and environment points in a single window."""

    def __init__(self):
        self.frame_count = 0
        self._lock       = threading.Lock()
        self._escape     = False

        self.det_points  = deque(maxlen=MAP_MAX_POINTS)
        self.det_colors  = deque(maxlen=MAP_MAX_POINTS)
        self.env_points  = deque(maxlen=MAP_MAX_POINTS)
        self.env_colors  = deque(maxlen=MAP_MAX_POINTS)

        self.det_pcd     = o3d.geometry.PointCloud()
        self.env_pcd     = o3d.geometry.PointCloud()  
        self.vis         = o3d.visualization.VisualizerWithKeyCallback()

        self._thread = threading.Thread(target=self._run_visualizer, daemon=True)
        self._thread.start()

    def _on_escape(self, vis):
        self._escape = True
        return False

    def _run_visualizer(self):
        self.vis.create_window(window_name="Combined Map", width=960, height=540)
        self.vis.register_key_callback(256, self._on_escape)

        render_opt = self.vis.get_render_option()
        render_opt.point_size       = MAP_POINT_SIZE
        render_opt.background_color = np.array([0.1, 0.1, 0.1])

        coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=0.3, origin=[0, 0, 0]
        )
        self.vis.add_geometry(coord_frame)

        det_pcd_added = False
        env_pcd_added = False

        while not self._escape:
            with self._lock:
                if len(self.det_points) > 0:
                    self.det_pcd.points = o3d.utility.Vector3dVector(np.array(self.det_points))
                    self.det_pcd.colors = o3d.utility.Vector3dVector(np.array(self.det_colors))
                    if not det_pcd_added:
                        self.vis.add_geometry(self.det_pcd)
                        det_pcd_added = True
                    else:
                        self.vis.update_geometry(self.det_pcd)

                if len(self.env_points) > 0:
                    self.env_pcd.points = o3d.utility.Vector3dVector(np.array(self.env_points))
                    self.env_pcd.colors = o3d.utility.Vector3dVector(np.array(self.env_colors)) 
                    if not env_pcd_added:
                        self.vis.add_geometry(self.env_pcd)
                        env_pcd_added = True
                    else:
                        self.vis.update_geometry(self.env_pcd)

            if not self.vis.poll_events():
                break
            self.vis.update_renderer()

    def update(self, detections, depth_image):
        self.frame_count += 1
        if self.frame_count % MAP_UPDATE_EVERY_N != 0:
            return

        with self._lock:
            
            for det in detections:
                if det.distance_cm <= 0.0:
                    continue
                X, Y, Z = pixel_to_3d(det.cx, det.cy, det.distance_cm)
                if Z <= 0.0 or Z > 10.0:
                    continue
                self.det_points.append([X, Y, Z])
                self.det_colors.append(CLASS_COLORS.get(det.label, DEFAULT_COLOR))

            if depth_image is not None:
                step      = MAP_ENV_SUBSAMPLE
                depth_sub = depth_image[::step, ::step].astype(np.float32)

                mask = np.ones(depth_image.shape, dtype=bool)
                for det in detections:
                    mask[det.y1:det.y2, det.x1:det.x2] = False
                mask_sub = mask[::step, ::step]

                valid_mask = (depth_sub > 0) & mask_sub
                rows, cols = np.where(valid_mask)
                rows_orig  = rows * step
                cols_orig  = cols * step

                if len(rows_orig) > 0:
                    depths  = depth_sub[rows, cols]
                    depth_m = depths * 0.001

                    X = -((cols_orig - CAM_PPX) * depth_m / CAM_FX)
                    Y = -((rows_orig - CAM_PPY) * depth_m / CAM_FY)
                    Z = depth_m

                    range_mask = (Z > 0.0) & (Z <= 10.0)
                    X = X[range_mask]
                    Y = Y[range_mask]
                    Z = Z[range_mask]

                    if len(X) > 0:
                        new_points = np.stack([X, Y, Z], axis=1)
                        new_colors = np.tile(MAP_ENV_COLOR, (len(X), 1))
                        self.env_points.extend(new_points.tolist())
                        self.env_colors.extend(new_colors.tolist())

    def escaped(self):
        return self._escape

    def stop(self):
        try:
            self.vis.destroy_window()  
        except Exception:
            pass

    def clear(self):
        with self._lock:              
            self.det_points.clear()
            self.det_colors.clear()
            self.env_points.clear()
            self.env_colors.clear()
        self.frame_count = 0
        self._escape     = False


# ─────────────────────────────────────────
# MENU
# ─────────────────────────────────────────
def show_menu():
    """Displays the map mode menu and returns the chosen mapper."""
    print("\n" + "-" * 40)
    print(" POINT MAP - Window Options")
    print("-" * 40)
    print(" 1 - Detection map only")
    print(" 2 - Environment map only")
    print(" 3 - Both maps, separate windows")
    print(" 4 - Both maps, combined window")
    print(" q - Quit")
    print("-" * 40)

    while True:
        choice = input("    Choose an option: ").strip().lower()
        if choice == "1":
            return "detection"
        elif choice == "2":
            return "environment"
        elif choice == "3":
            return "separate"
        elif choice == "4":
            return "combined"
        elif choice == "q":
            return "quit"
        else:
            print("  Invalid option. Please enter 1, 2, 3, 4 or q.")