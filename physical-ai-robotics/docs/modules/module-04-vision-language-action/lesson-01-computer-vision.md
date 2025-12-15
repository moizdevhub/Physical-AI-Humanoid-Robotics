---
id: lesson-01-computer-vision
title: Lesson 1 - Computer Vision for Robots
sidebar_position: 2
---

# Lesson 1: Computer Vision for Robots

## Introduction

**Computer vision** is a robot's primary sense for understanding the physical world. For humanoid robots, vision enables object recognition, manipulation, navigation, and human-robot interaction. Unlike standalone CV applications, robotic vision must run in real-time, integrate with 3D geometry, and output actionable information for control systems.

In this lesson, you'll learn to process camera feeds with OpenCV, run object detection with YOLO, perform semantic segmentation, estimate object poses, and integrate vision pipelines with ROS 2.

## ROS 2 Vision Pipeline

### Camera Topics

ROS 2 uses standard message types for vision data:

**sensor_msgs/Image:**
- Raw camera frames (RGB, BGR, mono)
- Header with timestamp and frame ID
- Encoding (e.g., "bgr8", "rgb8", "mono8")

**sensor_msgs/CameraInfo:**
- Camera intrinsics (focal length, principal point)
- Distortion coefficients
- Required for 3D reconstruction

**sensor_msgs/PointCloud2:**
- 3D point clouds from depth cameras
- XYZRGB format (position + color)

**Example subscription:**
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

    def image_callback(self, msg):
        # Convert ROS Image to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Process image
        # ... object detection, segmentation, etc.

        # Publish results
        # ...
```

### cv_bridge

**cv_bridge** converts between ROS Image messages and OpenCV/NumPy arrays.

**ROS → OpenCV:**
```python
cv_image = bridge.imgmsg_to_cv2(ros_image, desired_encoding='bgr8')
# Returns: NumPy array (height, width, 3)
```

**OpenCV → ROS:**
```python
ros_image = bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
# Returns: sensor_msgs/Image
```

**Common encodings:**
- `'bgr8'`: OpenCV default (8-bit BGR)
- `'rgb8'`: RGB format
- `'mono8'`: Grayscale
- `'32FC1'`: Depth image (32-bit float)

## Image Processing with OpenCV

### Basic Operations

**Read camera info:**
```python
from sensor_msgs.msg import CameraInfo

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.camera_info = None

        self.info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            10
        )

    def camera_info_callback(self, msg):
        self.camera_info = msg
        self.fx = msg.k[0]  # Focal length x
        self.fy = msg.k[4]  # Focal length y
        self.cx = msg.k[2]  # Principal point x
        self.cy = msg.k[5]  # Principal point y
```

**Image filtering:**
```python
import cv2
import numpy as np

# Gaussian blur (noise reduction)
blurred = cv2.GaussianBlur(image, (5, 5), 0)

# Edge detection
edges = cv2.Canny(blurred, 50, 150)

# Morphological operations
kernel = np.ones((5,5), np.uint8)
eroded = cv2.erode(edges, kernel, iterations=1)
dilated = cv2.dilate(eroded, kernel, iterations=1)
```

**Color space conversion:**
```python
# BGR to HSV (better for color-based segmentation)
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# Color masking (e.g., detect red objects)
lower_red = np.array([0, 120, 70])
upper_red = np.array([10, 255, 255])
mask = cv2.inRange(hsv, lower_red, upper_red)

# Extract red regions
red_objects = cv2.bitwise_and(image, image, mask=mask)
```

**Contour detection:**
```python
# Find contours
contours, hierarchy = cv2.findContours(
    mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
)

# Filter by area
min_area = 500
large_contours = [c for c in contours if cv2.contourArea(c) > min_area]

# Draw bounding boxes
for contour in large_contours:
    x, y, w, h = cv2.boundingRect(contour)
    cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0), 2)
```

## Object Detection

### YOLO (You Only Look Once)

**State-of-the-art real-time object detector.**

**Key features:**
- Single-pass inference (fast)
- 80 object classes (COCO dataset)
- Bounding boxes + class + confidence
- YOLOv8: Latest version with improved accuracy

**Installation:**
```bash
pip3 install ultralytics opencv-python
```

**YOLOv8 in ROS 2:**
```python
#!/usr/bin/env python3
"""
YOLO object detection node for ROS 2
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2


class YOLODetectionNode(Node):
    """Real-time object detection with YOLOv8."""

    def __init__(self):
        super().__init__('yolo_detection_node')

        # Load YOLO model
        self.model = YOLO('yolov8n.pt')  # nano model (fastest)
        self.bridge = CvBridge()

        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Publishers
        self.detection_pub = self.create_publisher(
            Detection2DArray,
            '/detections',
            10
        )

        self.vis_pub = self.create_publisher(
            Image,
            '/detections/image',
            10
        )

        self.get_logger().info("YOLO detection node started")

    def image_callback(self, msg):
        """Process image and detect objects."""
        # Convert to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Run YOLO inference
        results = self.model(cv_image, verbose=False)

        # Create detection message
        detections_msg = Detection2DArray()
        detections_msg.header = msg.header

        # Process each detection
        for result in results:
            for box in result.boxes:
                # Extract detection info
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                confidence = float(box.conf[0])
                class_id = int(box.cls[0])
                class_name = self.model.names[class_id]

                # Skip low-confidence detections
                if confidence < 0.5:
                    continue

                # Create Detection2D message
                detection = Detection2D()
                detection.header = msg.header

                # Bounding box
                detection.bbox.center.position.x = float((x1 + x2) / 2)
                detection.bbox.center.position.y = float((y1 + y2) / 2)
                detection.bbox.size_x = float(x2 - x1)
                detection.bbox.size_y = float(y2 - y1)

                # Class hypothesis
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = class_name
                hypothesis.hypothesis.score = confidence
                detection.results.append(hypothesis)

                detections_msg.detections.append(detection)

                # Draw on image
                cv2.rectangle(cv_image,
                             (int(x1), int(y1)),
                             (int(x2), int(y2)),
                             (0, 255, 0), 2)
                label = f"{class_name} {confidence:.2f}"
                cv2.putText(cv_image, label,
                           (int(x1), int(y1)-10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Publish detections
        self.detection_pub.publish(detections_msg)

        # Publish visualization
        vis_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        self.vis_pub.publish(vis_msg)

        self.get_logger().info(f"Detected {len(detections_msg.detections)} objects")


def main(args=None):
    rclpy.init(args=args)
    node = YOLODetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**YOLO model comparison:**

| Model | Size | Speed (FPS) | mAP | Use Case |
|-------|------|-------------|-----|----------|
| YOLOv8n | 3 MB | 100+ | 37.3 | Real-time robotics |
| YOLOv8s | 11 MB | 80 | 44.9 | Balanced |
| YOLOv8m | 26 MB | 50 | 50.2 | High accuracy |
| YOLOv8l | 44 MB | 30 | 52.9 | Offline processing |

**For humanoid robots:** Use YOLOv8n for real-time performance.

### Faster R-CNN

**Two-stage detector (slower but more accurate).**

**Architecture:**
1. Region Proposal Network (RPN) generates candidate boxes
2. Classifier refines boxes and assigns classes

**When to use:**
- High-accuracy requirements
- Offline processing acceptable
- Small object detection

**PyTorch implementation:**
```python
import torch
import torchvision

# Load pre-trained Faster R-CNN
model = torchvision.models.detection.fasterrcnn_resnet50_fpn(pretrained=True)
model.eval()

def detect_objects(image):
    """Run Faster R-CNN inference."""
    # Convert to tensor
    image_tensor = torch.from_numpy(image).permute(2, 0, 1).float() / 255.0
    image_tensor = image_tensor.unsqueeze(0)  # Add batch dimension

    # Inference
    with torch.no_grad():
        predictions = model(image_tensor)

    # Extract results
    boxes = predictions[0]['boxes'].cpu().numpy()
    scores = predictions[0]['scores'].cpu().numpy()
    labels = predictions[0]['labels'].cpu().numpy()

    return boxes, scores, labels
```

## Semantic Segmentation

**Pixel-level classification** (e.g., floor, wall, table, person).

### DeepLabV3

**State-of-the-art segmentation model.**

**Architecture:**
- Atrous convolutions for multi-scale features
- 21 classes (PASCAL VOC) or 150 classes (ADE20K)
- Output: Segmentation mask (height × width)

**Implementation:**
```python
import torch
import torchvision.transforms as T
from torchvision.models.segmentation import deeplabv3_resnet50

class SegmentationNode(Node):
    """Semantic segmentation with DeepLabV3."""

    def __init__(self):
        super().__init__('segmentation_node')

        # Load model
        self.model = deeplabv3_resnet50(pretrained=True)
        self.model.eval()

        # Preprocessing
        self.transform = T.Compose([
            T.ToTensor(),
            T.Normalize(mean=[0.485, 0.456, 0.406],
                       std=[0.229, 0.224, 0.225])
        ])

        # Class colors (PASCAL VOC)
        self.colors = self.get_class_colors()

        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )

        self.seg_pub = self.create_publisher(
            Image, '/segmentation/mask', 10
        )

    def get_class_colors(self):
        """RGB colors for each class."""
        return {
            0: (0, 0, 0),       # Background
            1: (128, 0, 0),     # Aeroplane
            15: (0, 128, 128),  # Person
            # ... 21 classes total
        }

    def image_callback(self, msg):
        """Segment image."""
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # Preprocess
        input_tensor = self.transform(cv_image).unsqueeze(0)

        # Inference
        with torch.no_grad():
            output = self.model(input_tensor)['out'][0]

        # Get class per pixel
        output_predictions = output.argmax(0).byte().cpu().numpy()

        # Colorize mask
        h, w = output_predictions.shape
        color_mask = np.zeros((h, w, 3), dtype=np.uint8)
        for class_id, color in self.colors.items():
            color_mask[output_predictions == class_id] = color

        # Overlay on original image
        overlay = cv2.addWeighted(cv_image, 0.6, color_mask, 0.4, 0)

        # Publish
        seg_msg = self.bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
        self.seg_pub.publish(seg_msg)
```

**Use cases for robotics:**
- **Floor segmentation:** Identify navigable surfaces
- **Object segmentation:** Separate objects from background
- **Scene understanding:** Classify environment (indoor, outdoor, kitchen, etc.)

## Pose Estimation

### 6D Object Pose

**Estimate 3D position and orientation of objects.**

**Problem:** Given 2D bounding box, find 6DOF pose (x, y, z, roll, pitch, yaw).

**Approaches:**

**1. PnP (Perspective-n-Point):**
- Match 2D image points to 3D object model points
- Solve for camera-to-object transformation

```python
import cv2
import numpy as np

def estimate_6d_pose(image_points, object_points, camera_matrix, dist_coeffs):
    """
    Estimate 6D pose using PnP.

    Args:
        image_points: (N, 2) 2D points in image
        object_points: (N, 3) 3D points on object
        camera_matrix: (3, 3) camera intrinsics
        dist_coeffs: (5,) distortion coefficients

    Returns:
        rvec: (3,) rotation vector
        tvec: (3,) translation vector
    """
    success, rvec, tvec = cv2.solvePnP(
        object_points, image_points, camera_matrix, dist_coeffs
    )

    if not success:
        return None, None

    return rvec, tvec

# Example: Cube with known 3D corners
object_points = np.array([
    [0, 0, 0],      # Corner 1
    [0.1, 0, 0],    # Corner 2
    [0.1, 0.1, 0],  # Corner 3
    [0, 0.1, 0],    # Corner 4
], dtype=np.float32)

# Detected 2D corners in image
image_points = np.array([
    [120, 150],
    [180, 155],
    [175, 210],
    [115, 205],
], dtype=np.float32)

camera_matrix = np.array([
    [500, 0, 320],
    [0, 500, 240],
    [0, 0, 1]
], dtype=np.float32)

dist_coeffs = np.zeros(5)

rvec, tvec = estimate_6d_pose(image_points, object_points, camera_matrix, dist_coeffs)

# Convert rotation vector to rotation matrix
R, _ = cv2.Rodrigues(rvec)

print(f"Position: {tvec.flatten()}")
print(f"Rotation matrix:\n{R}")
```

**2. Deep learning (DenseFusion, PVN3D):**
- Neural networks predict 6D pose directly from RGB-D images
- More robust but requires training data

### Human Pose Estimation

**Detect human body keypoints** (shoulders, elbows, wrists, hips, knees, ankles).

**MediaPipe Pose:**
```python
import mediapipe as mp
import cv2

class HumanPoseNode(Node):
    """Human pose detection with MediaPipe."""

    def __init__(self):
        super().__init__('human_pose_node')

        # Initialize MediaPipe Pose
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose(
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )
        self.mp_draw = mp.solutions.drawing_utils

        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )

        self.pose_pub = self.create_publisher(
            Image, '/human_pose/image', 10
        )

    def image_callback(self, msg):
        """Detect human pose."""
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # Convert BGR to RGB
        rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

        # Process
        results = self.pose.process(rgb_image)

        # Draw pose landmarks
        if results.pose_landmarks:
            self.mp_draw.draw_landmarks(
                cv_image,
                results.pose_landmarks,
                self.mp_pose.POSE_CONNECTIONS
            )

            # Extract specific keypoints
            landmarks = results.pose_landmarks.landmark
            right_wrist = landmarks[self.mp_pose.PoseLandmark.RIGHT_WRIST]

            self.get_logger().info(
                f"Right wrist: ({right_wrist.x:.2f}, {right_wrist.y:.2f})"
            )

        # Publish
        pose_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        self.pose_pub.publish(pose_msg)
```

**Use cases:**
- **Human-robot collaboration:** Avoid collisions with humans
- **Gesture recognition:** Interpret hand signals
- **Activity recognition:** Understand what humans are doing

## Depth Estimation and 3D Reconstruction

### Depth from Stereo

**Disparity calculation from stereo camera pair.**

**Principle:**
- Same point appears at different x-coordinates in left/right images
- Disparity: `d = x_left - x_right`
- Depth: `Z = (f × B) / d`
  - f: focal length
  - B: baseline (distance between cameras)

**OpenCV stereo matching:**
```python
import cv2
import numpy as np

class StereoDepthNode(Node):
    """Compute depth from stereo images."""

    def __init__(self):
        super().__init__('stereo_depth_node')

        # Stereo matcher
        self.stereo = cv2.StereoBM_create(numDisparities=64, blockSize=15)

        self.left_sub = self.create_subscription(
            Image, '/camera/left/image_raw', self.left_callback, 10
        )
        self.right_sub = self.create_subscription(
            Image, '/camera/right/image_raw', self.right_callback, 10
        )

        self.depth_pub = self.create_publisher(
            Image, '/depth/image', 10
        )

        self.left_image = None
        self.right_image = None

    def left_callback(self, msg):
        self.left_image = self.bridge.imgmsg_to_cv2(msg, 'mono8')
        self.compute_depth()

    def right_callback(self, msg):
        self.right_image = self.bridge.imgmsg_to_cv2(msg, 'mono8')
        self.compute_depth()

    def compute_depth(self):
        """Compute disparity map."""
        if self.left_image is None or self.right_image is None:
            return

        # Compute disparity
        disparity = self.stereo.compute(self.left_image, self.right_image)

        # Convert to depth (assuming known calibration)
        focal_length = 500  # pixels
        baseline = 0.1      # meters
        depth = (focal_length * baseline) / (disparity + 1e-6)

        # Normalize for visualization
        depth_vis = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)

        # Publish
        depth_msg = self.bridge.cv2_to_imgmsg(depth_vis, encoding='mono8')
        self.depth_pub.publish(depth_msg)
```

### Monocular Depth Estimation

**Estimate depth from single RGB image using deep learning.**

**MiDaS (Monocular Depth Estimation):**
```python
import torch

class MonocularDepthNode(Node):
    """Depth estimation from single image."""

    def __init__(self):
        super().__init__('monocular_depth_node')

        # Load MiDaS model
        self.model = torch.hub.load('intel-isl/MiDaS', 'MiDaS_small')
        self.model.eval()

        # Load transforms
        midas_transforms = torch.hub.load('intel-isl/MiDaS', 'transforms')
        self.transform = midas_transforms.small_transform

        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )

        self.depth_pub = self.create_publisher(
            Image, '/depth/image', 10
        )

    def image_callback(self, msg):
        """Estimate depth."""
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # Preprocess
        input_batch = self.transform(cv_image)

        # Inference
        with torch.no_grad():
            prediction = self.model(input_batch)
            prediction = torch.nn.functional.interpolate(
                prediction.unsqueeze(1),
                size=cv_image.shape[:2],
                mode='bicubic',
                align_corners=False
            ).squeeze()

        depth_map = prediction.cpu().numpy()

        # Normalize for visualization
        depth_vis = cv2.normalize(depth_map, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
        depth_vis = cv2.applyColorMap(depth_vis, cv2.COLORMAP_MAGMA)

        # Publish
        depth_msg = self.bridge.cv2_to_imgmsg(depth_vis, encoding='bgr8')
        self.depth_pub.publish(depth_msg)
```

### 3D Point Cloud from RGB-D

**Combine RGB + Depth → 3D point cloud.**

```python
def depth_to_pointcloud(depth_image, rgb_image, camera_info):
    """
    Convert depth image to 3D point cloud.

    Args:
        depth_image: (H, W) depth in meters
        rgb_image: (H, W, 3) RGB image
        camera_info: Camera intrinsics

    Returns:
        points: (N, 3) XYZ coordinates
        colors: (N, 3) RGB colors
    """
    h, w = depth_image.shape
    fx = camera_info.k[0]
    fy = camera_info.k[4]
    cx = camera_info.k[2]
    cy = camera_info.k[5]

    # Create pixel grid
    u, v = np.meshgrid(np.arange(w), np.arange(h))

    # Backproject to 3D
    z = depth_image
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy

    # Stack into points
    points = np.stack([x, y, z], axis=-1).reshape(-1, 3)
    colors = rgb_image.reshape(-1, 3)

    # Filter invalid depths
    valid = points[:, 2] > 0
    points = points[valid]
    colors = colors[valid]

    return points, colors
```

## Key Takeaways

- **ROS 2 vision uses sensor_msgs/Image and cv_bridge for OpenCV integration**
- **YOLO provides real-time object detection (>100 FPS on GPU)**
- **Semantic segmentation classifies every pixel (floor, object, person)**
- **6D pose estimation combines 2D detections with 3D geometry**
- **Depth estimation: stereo cameras or monocular deep learning (MiDaS)**
- **Point clouds enable 3D manipulation and navigation**
- **vision_msgs/Detection2D is standard format for object detections**
- **Always process camera_info for accurate 3D reconstruction**

## Next Steps

In **[Lesson 2: Vision-Language-Action Models](lesson-02-vision-language-action.md)**, you will:
- Understand VLA model architectures (vision + language → actions)
- Learn about RT-1, RT-2, Octo, and OpenVLA
- Explore behavior cloning and imitation learning
- Understand action tokenization strategies
- Access training datasets (Open X-Embodiment, RoboNet)

**Before moving on**, make sure you can:
- Convert ROS Image messages to OpenCV format with cv_bridge
- Run YOLO object detection on camera feeds
- Understand the difference between detection and segmentation
- Compute 3D points from depth images using camera intrinsics

---

**Estimated time for Lesson 2:** 3 hours (VLA architectures + model review)
