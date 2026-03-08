IC Computer Vision V2
Real-time object detection, distance measurement, and 3D point mapping using an Intel RealSense D456 depth camera and YOLOv8.

Overview
This project captures aligned color and depth frames from the D456, runs YOLOv8 object detection on the color stream, measures the real-world distance to each detected object, and maps detections into a live 3D point cloud using Open3D.

Features

- Depth-aligned color streaming via Intel RealSense D456
- Object detection using YOLOv8 (COCO dataset, 80 classes)
- Distance measurement per detected object in cm
- Live 3D point mapping with Open3D (in progress)
- Centralized config — all parameters in a single config.py


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
bashgit clone https://github.com/your-username/ic-computer-visionV2.git
cd ic-computer-visionV2
2. Create and activate a virtual environment
bashpython -m venv .venv
source .venv/bin/activate
3. Install dependencies
bashpip install pyrealsense2 numpy opencv-python ultralytics open3d
4. Connect the D456 camera and run
bashcd src
python main.py

The YOLOv8 model (yolov8s.pt) will be downloaded automatically on first run (~22MB).


Configuration
All parameters are centralized in src/config.py:
ParameterDefaultDescriptionCAMERA_WIDTH1280Stream width in pixelsCAMERA_HEIGHT720Stream height in pixelsCAMERA_FPS30Frames per secondYOLO_MODEL_PATHyolov8s.ptYOLO model to useYOLO_CONFIDENCE0.6Minimum detection confidenceYOLO_IOU_THRESHOLD0.45NMS IoU thresholdYOLO_DEVICEcpucpu, cuda, or mpsDEPTH_MIN_CM20.0Minimum reliable depth (cm)DEPTH_MAX_CM1000.0Maximum reliable depth (cm)MAP_MAX_POINTS10,000Max points in the live 3D mapSHOW_DEPTH_WINDOWFalseToggle depth colormap window

Usage
Run the full pipeline
bashpython main.py
Run the detection test
bashpython test.py
Controls
KeyActionq or ESCQuit

Roadmap

STATUS:
IN PROGRESS - IN INITIAL STAGES OR NOT YET STARTED
OPTIMIZE - RUNS AND WORKS, BUT PROBABLY STILL ROOM FOR OPTIMIZATION
COMPLETE - COMPLETE, MOST CHANGES ARE MINOR IMPROVEMENTS, QUALITY OF LIFE OR COSMETIC

 RealSense D456 camera interface (COMPLETE)
 Depth measurement at pixel (OPTIMIZE)
 YOLOv8 object detection (OPTIMIZE)
 Distance per detection in overlay (OPTIMIZE)
 Live 3D point mapping with Open3D (IN PROGRESS)
 Main pipeline integration with point map (IN PROGRESS)


License
This project is for research and educational purposes.