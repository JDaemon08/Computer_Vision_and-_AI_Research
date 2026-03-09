
#----------------------
# CAMERA
#----------------------
CAMERA_WIDTH    = 1280
CAMERA_HEIGHT   = 720
CAMERA_FPS      = 30

#----------------------
# YOLO
#----------------------
YOLO_MODEL_PATH     = "yolov8s.pt" # n=nano, s=small, m=medium, l=large - Larger models are more accurate, but heavier and more difficult to run.
YOLO_CONFIDENCE     = 0.6 #minimum confidence to accept detection
YOLO_IOU_THRESHOLD  = 0.45 # IoU threshold for NMS
YOLO_INPUT_SIZE     = 640 #YOLO inference resolution
YOLO_DEVICE         = "cpu"

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

#----------------------
# VISUALIZATION (OpenCV overlay)
#----------------------
BBOX_COLOR          = (0, 255, 0)    # BGR
BBOX_THICKNESS      = 2
LABEL_COLOR         = (0, 255, 0)
LABEL_FONT_SCALE    = 0.5
LABEL_THICKNESS     = 1
SHOW_DEPTH_WINDOW   = False           # toggle the depth colormap window

