import pyrealsense2 as rs
import cv2
from get_depth import RealSenseCamera
from detector import ObjectDetector

ctx = rs.context()
devices = ctx.query_devices()

if not devices:
    raise RuntimeError("Nenhum dispositivo RealSense detectado. Verifique a conexão!")

for dev in devices:
    print(f"Dispositivo detectado: {dev.get_info(rs.camera_info.name)}")

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

if __name__ == "__main__":
    test_detector()