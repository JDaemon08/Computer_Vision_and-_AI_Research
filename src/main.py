import cv2
from get_depth import RealSenseCamera
from detector import ObjectDetector
from imu import IMUTracker
from point_map import (
    PointMapper,
    EnvironmentMapper,
    CombinedMapper,
    show_menu
)
from config import (
    BBOX_COLOR,
    BBOX_THICKNESS,
    LABEL_COLOR,
    LABEL_FONT_SCALE,
    LABEL_THICKNESS
)

def build_mapper(choice):
    """Instantiates the correct mapper based on menu choice"""
    if choice == "detection":
        return PointMapper(), None
    elif choice == "environment":
        return None, EnvironmentMapper()
    elif choice == "separate":
        return PointMapper(), EnvironmentMapper(start_delay=2.0)
    elif choice == "combined":
        return CombinedMapper(), None
    return None, None  

def run_loop(camera, detector, choice):
    det_mapper, env_mapper = build_mapper(choice)
    combined = isinstance(det_mapper, CombinedMapper)

    try:
        while True:
            try:
                color_frame, depth_frame = camera.get_frames()
            except Exception as e:
                print(f"\n[Camera Error] {e}")
                print("Camera may have been disconnected. Returning to menu...")
                return True 

            if color_frame is None:
                continue

            try:
                detections = detector.detect(color_frame)
            except Exception as e:
                print(f"\n[Detector Error] {e}")
                continue

            for det in detections:
                try:
                    det.distance_cm = camera.depth_at_pixel(det.cx, det.cy, depth_frame)
                except Exception as e:
                    print(f"\n[Depth Error] {e}")
                    det.distance_cm = 0.0

                if det.distance_cm == 0.0:
                    continue

                cv2.rectangle(color_frame, (det.x1, det.y1), (det.x2, det.y2),
                              BBOX_COLOR, BBOX_THICKNESS)

                if det.track_id != -1:
                    label_text = f"[{det.track_id}] {det.label} {det.confidence:.0%} | {det.distance_cm:.1f} cm"
                else:
                    label_text = f"{det.label} {det.confidence:.0%} | {det.distance_cm:.1f} cm"

                cv2.putText(color_frame, label_text,
                            (det.x1, det.y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            LABEL_FONT_SCALE, LABEL_COLOR, LABEL_THICKNESS)

            try:
                valid_detections = [d for d in detections if d.distance_cm > 0.0]
                if combined:
                    det_mapper.update(valid_detections, depth_frame)
                else:
                    if det_mapper:
                        det_mapper.update(valid_detections)
                    if env_mapper:
                        env_mapper.update(depth_frame, valid_detections)
            except Exception as e:
                print(f"\n[Mapper Error] {e}")

            camera.show_image(color_frame, depth_frame)

            escaped = (
                (det_mapper and det_mapper.escaped()) or
                (env_mapper and env_mapper.escaped())
            )
            if escaped:
                print("\nReturning to menu...")
                return True

            if cv2.waitKey(1) & 0xFF in (ord('q'), 27):
                return False 

    except Exception as e:
        print(f"\n[Unexpected Error] {e}")
        print("Returning to menu...")
        return True

    finally:
        if det_mapper:
            det_mapper.stop()
        if env_mapper:
            env_mapper.stop()
        cv2.destroyAllWindows()

def main():
    camera   = RealSenseCamera()
    imu      = IMUTracker()
    camera.attach_imu(imu)
    detector = ObjectDetector()
    camera.start()

    try:
        while True:
            choice = show_menu()
            if choice == "quit":
                break
            should_continue = run_loop(camera, detector, choice)
            if not should_continue:
                break

    finally:
        camera.stop()
        print("Program closed.")


if __name__ == "__main__":
    main()