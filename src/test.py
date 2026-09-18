import pyrealsense2 as rs
import numpy as np
import cv2
import time
from get_depth import RealSenseCamera
from detector import ObjectDetector

#--------------------------------------------------------
# DEVICE CHECK
#--------------------------------------------------------
ctx = rs.context()
devices = ctx.query_devices()

if not devices:
    raise RuntimeError("No device detected! Check connection!")

for dev in devices:
    print(f"Device Detected: {dev.get_info(rs.camera_info.name)}")

#-------------------------------------------------------
# DETECTOR TEST
#-------------------------------------------------------

def test_detector():
    camera = RealSenseCamera()
    camera.start()
    detector =ObjectDetector()

    print("Running detection test - Press 'q' to quit\n" )

    try:
        while True:
            color_frame, depth_frame = camera.get_frames()

            if color_frame is None:
                continue

            detections = detector.detect(color_frame)

            for det in detections:
                cv2.rectangle(color_frame, (det.x1, det.y1), (det.x2, det.y2), (0,255,0), 2)
                cv2.putText(
                    color_frame,
                    f"{det.label} {det.confidence:.0%}",
                    (det.x1, det.y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0,255,0), 1
                )
            
            if detections:
                print(f"Detected {len(detections)} object(s):")
                for det in detections:
                    print(f" {det.label} ({det.confidence:.0%}) at center ({det.cx}, {det.cy})")
            else:
                print("No detections.")

            cv2.imshow("Detection Test", color_frame)

            if cv2.waitKey(1) & 0xFF in (ord('q'), 27):
                break
    finally:
        camera.stop()
        cv2.destroyAllWindows()


#---------------------------------------------------------
# IMU
#---------------------------------------------------------

def test_imu():
    pipeline = rs.pipeline()
    config = rs.config()

    config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 100)
    config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200)

    pipeline.start(config)
    print("\n Reading IMU data for 5 seconds - Move de camera around... \n")

    start = time.time()
    try:
        while time.time() - start < 5.0:
            frames     = pipeline.wait_for_frames()
            gyro_frame = frames.first_or_default(rs.stream.gyro)
            accel_frame = frames.first_or_default(rs.stream.accel)

            if gyro_frame:
                gyro = gyro_frame.as_motion_frame().get_motion_data()
                print(f"Gyro | x: {gyro.x:+.4f} y: {gyro.y:+.4f} z:{gyro.z:+.4f} rad/s")
            if accel_frame:
                accel = accel_frame.as_motion_frame().get_motion_data()
                print(f"Gyro | x: {accel.x:+.4f} y: {accel.y:+.4f} z:{accel.z:+.4f} m/s")

    finally:
        pipeline.stop()
        print("\nIMU test done")


#---------------------------------------------------------
# DEPTH AT CENTER
#---------------------------------------------------------

def test_depth():
    camera = RealSenseCamera()
    camera.start()

    print("\nRunning depth test — press 'q' to quit\n")

    try:
        while True:
            color_frame, depth_frame = camera.get_frames()

            if color_frame is None:
                continue

            cx = color_frame.shape[1] // 2
            cy = color_frame.shape[0] // 2
            dist = camera.depth_at_pixel(cx, cy, depth_frame)

            cv2.circle(color_frame, (cx, cy), 5, (0, 255, 0), -1)
            cv2.putText(color_frame, f"{dist:.1f} cm", (cx + 10, cy),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            cv2.imshow("Depth Test", color_frame)
            print(f"Distance at center: {dist:.1f} cm")

            if cv2.waitKey(1) & 0xFF in (ord('q'), 27):
                break

    finally:
        camera.stop()
        cv2.destroyAllWindows()

#----------------------------------------------------------
# RUN - Change whichtest to run here with comments
#----------------------------------------------------------

if __name__ == "__main__":
    #test_detector()
    test_imu()
    #test_depth()




