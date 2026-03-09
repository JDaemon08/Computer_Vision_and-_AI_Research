import cv2
from get_depth import RealSenseCamera
from detector import ObjectDetector
from config import (
    BBOX_COLOR,
    BBOX_THICKNESS,
    LABEL_COLOR,
    LABEL_FONT_SCALE,
    LABEL_THICKNESS
)

def main():
    camera   = RealSenseCamera()
    detector = ObjectDetector()

    camera.start()

    print("Running - Press 'q' to quit.\n" )

    try:
        while True:
            color_frame, depth_frame = camera.get_frames()

            if color_frame is None:
                continue

            detections = detector.detect(color_frame)

            for det in detections:
                det.distance_cm = camera.depth_at_pixel(det.cx, det.cy, depth_frame)

                if det.distance_cm == 0.0:
                    continue

                cv2.rectangle(
                    color_frame,
                    (det.x1, det.y1),
                    (det.x2, det.y2),
                    BBOX_COLOR, BBOX_THICKNESS
                )

                if det.track_id != -1:
                    label_text = f"[{det.track_id}] {det.label} {det.confidence:.0%} | {det.distance_cm:.1f} cm"
                else:
                    label_text = f"{det.label} {det.confidence:.0%} | {det.distance_cm:.1f} cm."
                    
                cv2.putText(
                    color_frame,
                    label_text,
                    (det.cx, det.cy - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    LABEL_FONT_SCALE,
                    LABEL_COLOR,
                    LABEL_THICKNESS
                )
            camera.show_image(color_frame, depth_frame)

            if cv2.waitKey(1) & 0xFF in (ord('q'), 27):
                print("Exiting...")
                break
    finally:
        camera.stop()
        cv2.destroyAllWindows()
        print('Program Closed.')

if __name__ == "__main__":
    main()