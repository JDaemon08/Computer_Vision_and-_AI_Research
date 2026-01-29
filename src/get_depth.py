import pyrealsense2 as rs
import numpy as np
import cv2

class RealSenseCamera:
    """Intel RealSense D456 Interface"""

    def __init__(self, width=1280, height=720, fps=30):
        #initializes and configures camera

        print('Initiating RealSense...')
        self.width = width
        self.height = height
        self.fps = fps

        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        self.config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)

        self.align_to = rs.align(rs.stream.color)

        self.frames = None

    def start(self):
        self.pipeline = rs.pipeline()
        self.profile = self.pipeline.start(self.config) # Start the internal pipeline
            
    def stop(self):
        print("Stopping pipeline...")
        self.pipeline.stop()
        print("Pipeline stopped;")
    
    def get_frames(self):
        #Captures and aligns new set of frames.
        try:
            frames = self.pipeline.wait_for_frames()
            aligned_frames = self.align_to.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

            #verificação
            if not depth_frame or not color_frame:
                print("Warning: Invalid color or depth frame.")
                return None, None
            
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            self.frames = (color_image, depth_image)
            return color_image, depth_image
        
        except Exception as e:
            print(f"[RealSenseCamera] failed to capture *frames*: {e}")
            return None, None
            
    def depth_at_pixel(self, x, y, depth_image=None):
        if depth_image is None:
            if self.frames is None:
                print("Warning: No depth frame available.")
        if y < 0 or y >= depth_image.shape[0] or x < 0 or x >= depth_image.shape[1]:
            print("Coordinates outside limits.")
            return 0.0
        
        #Depth scale for more acurrate measuring
        self.depth_scale = self.profile.get_device().first_depth_sensor().get_depth_scale()
        raw_depth = depth_image[y,x]
        distance_cm = raw_depth * self.depth_scale * 100

        return distance_cm
        
    def show_image(self, color_image, depth_image, window_name="RealSense"):
        if color_image is None or depth_image is None:
            if self.frames is None:
                print("There are no frames to show.")
                return
            color_image, depth_image = self.frames
        
        depth_colormap = cv2.applyColorMap(
            cv2.convertScaleAbs(depth_image, alpha=0.03),
            cv2.COLORMAP_JET
        )
        cv2.imshow(f'{window_name} - Color', color_image)
        cv2.imshow(f'{window_name} - Depth', depth_colormap)
        def __del__(self):
            self.stop()
    pass

if __name__ == "__main__":
    camera = RealSenseCamera()
    camera.start()

    try:
        while True:
            color_frame, depth_frame = camera.get_frames()

            if color_frame is None:
                continue

            #Posible Yolo Integration here

            camera.show_image(color_frame, depth_frame)

            center_x = color_frame.shape[1] // 2
            center_y = color_frame.shape[0] // 2
            distance = camera.depth_at_pixel(center_x, center_y, depth_frame)

            print(f'Distance at center ({center_x}, {center_y}): {distance:.0f} mm')

            key = cv2.waitKey(1) & 0xFF

            if key == ord('q') or key == 27:
                print("Exiting...")
                break
    
    finally:        #Depth scale for more acurrate measuring
        camera.stop()
        cv2.destroyAllWindows()
        print("Program Closed")