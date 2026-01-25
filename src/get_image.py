import pyrealsense2 as rs
import numpy as np
import cv2

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.infrared, 640, 480, rs.format.y8, 30)

print("Starting pipeline...")
profile = pipeline.start(config)


try:
    while True:
        #wait for the next frame
        frames = pipeline.wait_for_frames()
        print("Getting data...")
        
        #separate frames types
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        infrared_frame = frames.get_infrared_frame()

        if not depth_frame or not color_frame or not infrared_frame:
            continue
        
        #convert into array for OpenCv
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        infrared_image = np.asanyarray(infrared_frame.get_data())

        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        #display image
        cv2.imshow('RGB Feed',color_image)
        cv2.imshow('Depth Feed',depth_image)
        cv2.imshow('Infrared Feed', infrared_image)

        key = cv2.waitKey(1)
        if key & 0xff == ord('q') or key == 27:
            print("Quitting...")
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()