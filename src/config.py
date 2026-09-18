
#----------------------
# CAMERA
#----------------------
CAMERA_WIDTH    = 1280
CAMERA_HEIGHT   = 720
CAMERA_FPS      = 30

# ─────────────────────────────────────────
# CAMERA INTRINSICS (D456 @ 1280x720)
# ─────────────────────────────────────────
CAM_FX  = 643.9603271484375
CAM_FY  = 642.9613037109375
CAM_PPX = 656.1786499023438
CAM_PPY = 367.67108154296875

#----------------------
# YOLO
#----------------------
YOLO_MODEL_PATH     = "yolov8s.pt" # n=nano, s=small, m=medium, l=large - Larger models are more accurate, but heavier and more difficult to run.
YOLO_CONFIDENCE     = 0.45 #minimum confidence to accept detection
YOLO_IOU_THRESHOLD  = 0.35 # IoU threshold for NMS
YOLO_INPUT_SIZE     = 640 #YOLO inference resolution
YOLO_DEVICE         = "cpu"

#----------------------
# TRACKING
#----------------------
YOLO_TRACKING = True
YOLO_TRACKER  = "bytetrack.yaml" #or "botsort.yaml"

#----------------------
# DEPTH
#----------------------
DEPTH_MIN_CM = 50.0
DEPTH_MAX_CM = 1000.0
DEPTH_PATCH_SZ = 3

#----------------------
# POINT MAPPING
#----------------------
MAP_MAX_POINTS      = 10_000
MAP_POINT_SIZE      = 3.0
MAP_UPDATE_EVERY_N  = 3
MAP_ENV_ENABLED     = True    # toggle environment map on/off
MAP_ENV_SUBSAMPLE   = 8       # sample every Nth pixel (lower = denser, slower)
MAP_ENV_COLOR       = [0.4, 0.8, 1.0] 

#----------------------
# VISUALIZATION (OpenCV overlay)
#----------------------
BBOX_COLOR          = (0, 255, 0)    # BGR
BBOX_THICKNESS      = 2
LABEL_COLOR         = (0, 255, 0)
LABEL_FONT_SCALE    = 0.5
LABEL_THICKNESS     = 1
SHOW_DEPTH_WINDOW   = False           # toggle the depth colormap window

