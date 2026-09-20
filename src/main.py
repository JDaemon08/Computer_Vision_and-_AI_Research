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
    LABEL_THICKNESS,
    YOLO_FRAME_SKIP
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

def run_loop(camera, detector, choice, imu=None):
    det_mapper, env_mapper = build_mapper(choice)
    combined     = isinstance(det_mapper, CombinedMapper)
    frame_count  = 0
    last_detections = []

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

            rotation_matrix = imu.get_rotation_matrix() if imu else None

            if det_mapper:
                det_mapper.set_rotation(rotation_matrix)
            if env_mapper:
                env_mapper.set_rotation(rotation_matrix)

            frame_count += 1
            if frame_count % YOLO_FRAME_SKIP == 0:
                try:
                    last_detections = detector.detect(color_frame)
                except Exception as e:
                    print(f"\n[Detector Error] {e}")

            detections = last_detections

            for det in detections:
                try:
                    det.distance_cm = camera.depth_at_pixel(det.cx, det.cy, depth_frame)
                except Exception as e:
                    print(f"\n[Depth Error] {e}")
                    det.distance_cm = 0.0

                if det.distance_cm == 0.0:
                    continue

                # Bounding box
                cv2.rectangle(
                    color_frame,
                    (det.x1, det.y1), (det.x2, det.y2),
                    BBOX_COLOR, BBOX_THICKNESS
                )

                # Label
                if det.track_id != -1:
                    label_text = f"[{det.track_id}] {det.label} {det.confidence:.0%} | {det.distance_cm:.1f} cm"
                else:
                    label_text = f"{det.label} {det.confidence:.0%} | {det.distance_cm:.1f} cm"

                cv2.putText(
                    color_frame, label_text,
                    (det.x1, det.y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    LABEL_FONT_SCALE, LABEL_COLOR, LABEL_THICKNESS
                )

            try:
                valid_detections = [d for d in detections if d.distance_cm > 0.0]
                if combined:
                    det_mapper.update(valid_detections, depth_frame, rotation_matrix)
                else:
                    if det_mapper:
                        det_mapper.update(valid_detections, rotation_matrix)
                    if env_mapper:
                        env_mapper.update(depth_frame, valid_detections, rotation_matrix)
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

            # ── q or ESC on OpenCV window → full quit ─────────
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
            should_continue = run_loop(camera, detector, choice, imu)
            if not should_continue:
                break

    finally:
        camera.stop()
        print("Program closed.")


if __name__ == "__main__":
    main()