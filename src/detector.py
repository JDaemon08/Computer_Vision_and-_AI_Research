import cv2
from ultralytics import YOLO
from dataclasses import dataclass
from config import (
    YOLO_MODEL_PATH,
    YOLO_CONFIDENCE,
    YOLO_IOU_THRESHOLD,
    YOLO_INPUT_SIZE,
    YOLO_DEVICE
)

@dataclass
class Detection:
    """Single Object Detection."""
    label:      str
    confidence: float
    cx:         int     #center x
    cy:         int     #center y
    x1:         int     #bounding box: upper left
    y1:         int
    x2:         int     #bounding box: lower right
    y2:         int
    distance_cm: float = 0.0 #filled in later at main.py

class ObjectDetector:
    """Yolov8 wrapper for object detection;"""

    def __init__(self):
        print(f"Loading YOLO model from '{YOLO_MODEL_PATH}'...")
        self.model = YOLO(YOLO_MODEL_PATH)
        self.device = YOLO_DEVICE
        print(f"YOLO loaded. Running on: {self.device}")

    def detect(self, color_frame) -> list[Detection]:

        """
        Runs YOLOv8 interference with color frame.
        Returns a list of Detection Objects.
        """

        if color_frame is None:
            return []
        
        results = self.model.predict(
            source=color_frame,
            conf=YOLO_CONFIDENCE,
            iou=YOLO_IOU_THRESHOLD,
            imgsz=YOLO_INPUT_SIZE,
            device=self.device,
            verbose=False #suppress per-frame console output
        )
        
        detections = []

        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cx = (x1 + x2) // 2
                cy = (y1 + y2) // 2
                confidence = float(box.conf[0])
                label = self.model.names[int(box.cls[0])]

                detections.append(Detection(
                    label=label,
                    confidence=confidence,
                    cx=cx, cy=cy,
                    x1=x1, y1=y1,
                    x2=x2, y2=y2
                ))
        return detections