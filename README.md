IC Computer Vision V2

Real-time object detection, distance measurement, and 3D point mapping using an Intel RealSense D456 depth camera and YOLOv8.

Overview

This project captures aligned color and depth frames from the D456, runs YOLOv8 object detection on the color stream, measures the real-world distance to each detected object, and maps detections into a live 3D point cloud using Open3D.

Features
📷 Depth-aligned color streaming via Intel RealSense D456
🔍 Object detection using YOLOv8 (COCO dataset, 80 classes)
📏 Distance measurement per detected object in cm
🗺️ Live 3D point mapping with Open3D (in progress)
⚙️ Centralized config — all parameters in a single config.py
Project Structure
IC-COMPUTER-VISIONV2/
├── .venv/
├── src/
│   ├── config.py              # All settings in one place
│   ├── get_depth.py           # RealSense D456 interface
│   ├── detector.py            # YOLOv8 wrapper
│   ├── point_map.py           # 3D point cloud mapper (in progress)
│   ├── main.py                # Entry point
│   └── test.py                # Module tests
├── .gitignore
└── README.md
Requirements
Hardware
Intel RealSense D456 camera
Software
Python 3.10+
Intel RealSense SDK 2.0
Python Dependencies
pyrealsense2
numpy
opencv-python
ultralytics
open3d
Installation

1. Clone the repository

bash
git clone https://github.com/your-username/ic-computer-visionV2.git
cd ic-computer-visionV2

2. Create and activate a virtual environment

bash
python -m venv .venv
source .venv/bin/activate

3. Install dependencies

bash
pip install pyrealsense2 numpy opencv-python ultralytics open3d

4. Connect the D456 camera and run

bash
cd src
python main.py

The YOLOv8 model (yolov8s.pt) will be downloaded automatically on first run (~22MB).

Configuration

All parameters are centralized in src/config.py:

Parameter	Default	Description
CAMERA_WIDTH	1280	Stream width in pixels
CAMERA_HEIGHT	720	Stream height in pixels
CAMERA_FPS	30	Frames per second
YOLO_MODEL_PATH	yolov8s.pt	YOLO model to use
YOLO_CONFIDENCE	0.6	Minimum detection confidence
YOLO_IOU_THRESHOLD	0.45	NMS IoU threshold
YOLO_DEVICE	cpu	cpu, cuda, or mps
DEPTH_MIN_CM	20.0	Minimum reliable depth (cm)
DEPTH_MAX_CM	1000.0	Maximum reliable depth (cm)
MAP_MAX_POINTS	10,000	Max points in the live 3D map
SHOW_DEPTH_WINDOW	False	Toggle depth colormap window
Usage
Run the full pipeline
bash
python main.py
Run the detection test
bash
python test.py
Controls
Key	Action
q or ESC	Quit
Roadmap
 RealSense D456 camera interface
 Depth measurement at pixel
 YOLOv8 object detection
 Distance per detection in overlay
 Live 3D point mapping with Open3D
 Main pipeline integration with point map
License

This project is for research and educational purposes.