# IC Computer Vision V2

Real-time object detection, distance measurement, and 3D point mapping using an **Intel RealSense D456** depth camera and **YOLOv8**.

---

## Overview

This project captures aligned color and depth frames from the D456, runs YOLOv8 object detection on the color stream, measures the real-world distance to each detected object, and maps detections and the surrounding environment as a live 3D point cloud using Open3D. An IMU tracker reads the camera's built-in gyroscope and accelerometer to stabilize the point cloud as the camera rotates.

---

## Features

- **Depth-aligned color streaming** via Intel RealSense D456
- **Object detection** using YOLOv8s (COCO dataset, 80 classes)
- **Distance measurement** per detected object in cm
- **Live 3D point mapping** with Open3D — detection map, environment map, or both combined
- **IMU-assisted rotation** using the D456's built-in gyroscope and accelerometer
- **Centralized config** — all parameters in a single `config.py`

---

## Project Structure

```
IC-COMPUTER-VISIONV2/
├── .venv/
├── src/
│   ├── config.py        # All settings in one place
│   ├── get_depth.py     # RealSense D456 camera interface
│   ├── get_image.py     # Standalone test — RGB, depth and infrared streams
│   ├── detector.py      # YOLOv8 wrapper with ByteTrack tracking
│   ├── imu.py           # IMU tracker — gyro + accel to rotation matrix
│   ├── point_map.py     # 3D point cloud mappers + Open3D visualizer
│   ├── main.py          # Entry point — orchestrates all modules
│   ├── slam.py          # Slam module
│   └── test.py          # Module tests (detector, IMU, depth)
├── .gitignore
└── README.md
```

---

## Requirements

### Hardware

- Intel RealSense D456 camera

### Software

- Python 3.10+
- [Intel RealSense SDK 2.0](https://github.com/IntelRealSense/librealsense)

### Python Dependencies

```
pyrealsense2
numpy
opencv-python
ultralytics
open3d
```

---

## Installation

**1. Clone the repository**

```bash
git clone https://github.com/JDaemon08/ic-computer-visionV2.git
cd ic-computer-visionV2
```

**2. Create and activate a virtual environment**

```bash
python -m venv .venv
source .venv/bin/activate
```

**3. Install dependencies**

```bash
pip install pyrealsense2 numpy opencv-python ultralytics open3d
```

**4. Connect the D456 and run**

```bash
cd src
python main.py
```

> The YOLOv8 model (`yolov8s.pt`) downloads automatically on first run (~22MB).

---

## Configuration

All parameters are centralized in `src/config.py`:

| Parameter | Default | Description |
|---|---|---|
| `CAMERA_WIDTH` | 1280 | Stream width in pixels |
| `CAMERA_HEIGHT` | 720 | Stream height in pixels |
| `CAMERA_FPS` | 30 | Frames per second |
| `YOLO_MODEL_PATH` | `yolov8s.pt` | YOLO model to use |
| `YOLO_CONFIDENCE` | 0.45 | Minimum detection confidence |
| `YOLO_IOU_THRESHOLD` | 0.35 | NMS IoU threshold |
| `YOLO_DEVICE` | `cpu` | `cpu`, `cuda`, or `mps` |
| `YOLO_TRACKING` | `True` | Enable ByteTrack object tracking |
| `DEPTH_MIN_CM` | 50.0 | Minimum reliable depth (cm) |
| `DEPTH_MAX_CM` | 1000.0 | Maximum reliable depth (cm) |
| `DEPTH_PATCH_SZ` | 3 | Patch size for median depth sampling |
| `MAP_MAX_POINTS` | 10,000 | Rolling buffer size for point cloud |
| `MAP_ENV_SUBSAMPLE` | 8 | Environment map pixel sampling rate |
| `SHOW_DEPTH_WINDOW` | `False` | Toggle depth colormap window |

---

## Usage

### Run the full pipeline

```bash
python main.py
```

On startup, a menu lets you choose the point map mode:

```
 1 - Detection map only
 2 - Environment map only
 3 - Both maps, separate windows
 4 - Both maps, combined window
 q - Quit
```

Press **ESC** in any Open3D window to return to the menu.
Press **q** or **ESC** in the OpenCV window to exit the program.

### Run individual tests

```bash
python test.py
```

Edit the bottom of `test.py` to switch between `test_detector()`, `test_imu()`, and `test_depth()`.

---

## Roadmap

| Feature | Status |
|---|---|
| RealSense D456 camera interface | Complete |
| Depth measurement at pixel | Complete |
| YOLOv8 object detection | Complete |
| ByteTrack object tracking | Complete |
| Distance per detection in overlay | Complete |
| IMU gyroscope + accelerometer integration | Complete |
| Live 3D point mapping — detection | Complete |
| Live 3D point mapping — environment | Complete |
| IMU rotation applied to point cloud | In progress |
| SLAM integration (Open3D odometry) | Planned |

---

## License

This project is for research and educational purposes.