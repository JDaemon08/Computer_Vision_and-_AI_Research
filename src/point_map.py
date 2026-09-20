import open3d as o3d
import numpy as np
import threading
from collections import deque
from config import (
    CAM_FX, CAM_FY, CAM_PPX, CAM_PPY,
    CAMERA_WIDTH, CAMERA_HEIGHT,
    MAP_MAX_POINTS,
    MAP_POINT_SIZE,
    MAP_UPDATE_EVERY_N,
    MAP_ENV_ENABLED,
    MAP_ENV_SUBSAMPLE,
    MAP_ENV_COLOR,
    FRUSTRUM_DEPTH,
    FRUSTRUM_COLOR,
    FRUSTRUM_ORIGIN_COLOR
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


def pixel_to_3d(cx, cy, depth_cm, rotation_matrix=None):
    """
    Converts a pixel (cx, cy) and depth in cm
    to real world (X, Y, Z) coordinates in meters.
    """
    depth_m = depth_cm / 100.0
    X = -((cx - CAM_PPX) * depth_m / CAM_FX)  
    Y = -((cy - CAM_PPY) * depth_m / CAM_FY)  
    Z = depth_m

    if rotation_matrix is not None:
        point = np.array([X, Y, Z])
        point = rotation_matrix @ point
        X, Y, Z = point

    return X, Y, Z

class CameraFrustrum:
    """Renders live camera frustrum in Open3D window"""

    CORNERS = [
        (0,                        0),
        (CAMERA_WIDTH,             0),
        (0,            CAMERA_HEIGHT),
        (CAMERA_WIDTH, CAMERA_HEIGHT),
    ]

    def __init__(self):
        self.origin      = np.array([0.0, 0.0, 0.0])
        self.line_set    = o3d.geometry.LineSet()
        self.origin_mesh = o3d.geometry.TriangleMesh.create_sphere(radius=0.05)
        self.origin_mesh.paint_uniform_color(FRUSTRUM_ORIGIN_COLOR) 
        self._built       = False

    def _unprojected_corner(self, px, py, rotation_matrix):
        """Convert Pixel Corner to 3D ray direction"""

        rx = -(px - CAM_PPX) / CAM_FX
        ry = -(py - CAM_PPY) / CAM_FY
        rz = 1.0

        ray = np.array([rx, ry, rz])
        ray = ray / np.linalg.norm(ray)

        if rotation_matrix is not None:
            ray = rotation_matrix @ ray

        return ray * FRUSTRUM_DEPTH

    def get_points_and_lines(self, rotation_matrix):
        """Computes the 8 points and 8 lines of the frustrum"""

        tips = [
            self._unprojected_corner(px, py, rotation_matrix)
            for px, py in self.CORNERS
        ]

        forward = np.array([0.0, 0.0, 1.0])
        if rotation_matrix is not None:
            forward = rotation_matrix @ forward
        forward_tip = forward * FRUSTRUM_DEPTH

        points = [
            self.origin,
            tips[0],
            tips[1],
            tips[2],
            tips[3],
            forward_tip
        ]

        lines = [
            [0,1],
            [0,2],
            [0,3],
            [0,4],
            [1,2],
            [3,4],
            [1,3],
            [2,4],
            [0,5],
        ]

        colors = [FRUSTRUM_COLOR] * len(lines)

        return points, lines, colors

    def update (self, vis, rotation_matrix):
        points, lines, colors = self.get_points_and_lines(rotation_matrix)

        self.line_set.points = o3d.utility.Vector3dVector(np.array(points))
        self.line_set.lines  = o3d.utility.Vector2iVector(np.array(lines))
        self.line_set.colors = o3d.utility.Vector3dVector(np.array(colors))

        if not self._built:
            vis.add_geometry(self.line_set)
            vis.add_geometry(self.origin_mesh)
            self._built = True
        else:
            vis.update_geometry(self.line_set)
            vis.update_geometry(self.origin_mesh)
        

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
        self.frustrum    = CameraFrustrum()
        self._rotation   = None

        self._thread = threading.Thread(target=self._run_visualizer, daemon=True)
        self._thread.start()

    def set_rotation(self, rotation_matrix):
        with self._lock:
            self._rotation = rotation_matrix

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
                rotation = self._rotation

                if len(self.points) > 0:
                    self.pcd.points = o3d.utility.Vector3dVector(np.array(self.points))
                    self.pcd.colors = o3d.utility.Vector3dVector(np.array(self.colors))
                    if not pcd_added:
                        self.vis.add_geometry(self.pcd)
                        pcd_added = True
                    else:
                        self.vis.update_geometry(self.pcd)

            self.frustrum.update(self.vis, rotation)
            
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

    def update(self, detections, rotation_matrix=None):
        self.frame_count += 1
        if self.frame_count % MAP_UPDATE_EVERY_N != 0:
            return

        with self._lock:
            for det in detections:
                if det.distance_cm <= 0.0:
                    continue
                X, Y, Z = pixel_to_3d(det.cx, det.cy, det.distance_cm, rotation_matrix)
                if Z <= 0.0 or Z > 10.0:
                    continue
                self.points.append([X, Y, Z])
                self.colors.append(CLASS_COLORS.get(det.label, DEFAULT_COLOR))


class EnvironmentMapper(_BaseMapper):
    """Maps the environment as a dense point cloud, masking out detections."""

    def __init__(self, window_name="Environment Map", start_delay=0.5):
        self.start_delay = start_delay
        super().__init__(window_name, start_delay=0.0)

    def update(self, depth_image, detections, rotation_matrix=None):
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

        if rotation_matrix is not None:
            points = np.stack([X, Y, Z], axis=1)
            points = (rotation_matrix @ points.T).T
            X, Y, Z = points[:, 0], points[:, 1], points[:, 2]

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

        self.frustrum    = CameraFrustrum()
        self._rotation   - None

        self._thread = threading.Thread(target=self._run_visualizer, daemon=True)
        self._thread.start()

    def set_rotation(self, rotation_matrix):
        with self._lock:
            self._rotation = rotation_matrix

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
                rotation = self._rotation
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

            self.frustrum.update(self.vis, rotation)

            if not self.vis.poll_events():
                break
            self.vis.update_renderer()

    def update(self, detections, depth_image, rotation_matrix=None):
        self.frame_count += 1
        if self.frame_count % MAP_UPDATE_EVERY_N != 0:
            return

        with self._lock:
            for det in detections:
                if det.distance_cm <= 0.0:
                    continue
                X, Y, Z = pixel_to_3d(det.cx, det.cy, det.distance_cm, rotation_matrix)
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

                    if rotation_matrix is not None:
                        points = np.stack([X,Y,Z], axis=1)
                        points = (rotation_matrix @ points.T).T
                        X, Y, Z = points[:,0], points[:, 1], points[:, 2]

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