---
id: exercise-object-detection-grasping
title: Exercise - Object Detection and Language-Conditioned Grasping
sidebar_position: 5
---

# Exercise: Object Detection and Language-Conditioned Grasping

## Overview

In this comprehensive exercise, you'll build an end-to-end vision-language-action system for humanoid robot manipulation. You'll integrate YOLO object detection, CLIP language grounding, 3D grasp planning, and ROS 2 action servers to enable the robot to follow natural language commands like "pick up the red cube."

By the end, you'll have a complete VLA pipeline running on your simulated humanoid robot in Gazebo, demonstrating the state-of-the-art in robot

 manipulation.

**Estimated time:** 6-7 hours

## Learning Objectives

After completing this exercise, you will be able to:

✅ Stream and process camera images from Gazebo using cv_bridge
✅ Run real-time YOLO object detection on robot camera feeds
✅ Ground natural language queries to detected objects using CLIP
✅ Compute 3D grasp poses from 2D bounding boxes and depth maps
✅ Execute manipulation via ROS 2 action servers
✅ Visualize detections and grasp poses in RViz2
✅ Build complete VLA pipelines for language-conditioned manipulation

## Prerequisites

**Required:**
- Modules 1-3 completion (ROS 2, Gazebo, AI planning)
- Module 4 Lessons 1-3 completion
- Python 3.8+ with PyTorch

**Verify installation:**
```bash
ros2 pkg list | grep vision_msgs  # Vision message types
python3 -c "import torch; import clip"  # PyTorch and CLIP
python3 -c "from ultralytics import YOLO"  # YOLO
```

## Step 1: Install Dependencies

### Install Vision Packages

```bash
# ROS 2 vision packages
sudo apt install ros-humble-vision-msgs ros-humble-cv-bridge ros-humble-image-transport

# Python vision libraries
pip3 install ultralytics opencv-python pillow
pip3 install git+https://github.com/openai/CLIP.git

# PyTorch (if not already installed)
pip3 install torch torchvision
```

### Create Workspace

```bash
mkdir -p ~/humanoid_vla_ws/src
cd ~/humanoid_vla_ws/src

# Create VLA package
ros2 pkg create --build-type ament_python humanoid_vla \
  --dependencies rclpy sensor_msgs geometry_msgs vision_msgs std_msgs cv_bridge

cd humanoid_vla
mkdir scripts config launch
```

## Step 2: YOLO Object Detection Node

### Create Detection Node

Create `scripts/yolo_detector.py`:

```python
#!/usr/bin/env python3
"""
YOLO object detection node with visualization
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import numpy as np


class YOLODetector(Node):
    """Real-time object detection with YOLOv8."""

    def __init__(self):
        super().__init__('yolo_detector')

        # Parameters
        self.declare_parameter('model_name', 'yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('visualize', True)

        model_name = self.get_parameter('model_name').value
        self.conf_threshold = self.get_parameter('confidence_threshold').value
        self.visualize = self.get_parameter('visualize').value

        # Load YOLO model
        self.model = YOLO(model_name)
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

        if self.visualize:
            self.vis_pub = self.create_publisher(
                Image,
                '/detections/visualization',
                10
            )

        self.get_logger().info(f"YOLO detector started with model {model_name}")

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
                if confidence < self.conf_threshold:
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
                if self.visualize:
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
        if self.visualize:
            vis_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            self.vis_pub.publish(vis_msg)

        self.get_logger().info(f"Detected {len(detections_msg.detections)} objects")


def main(args=None):
    rclpy.init(args=args)
    node = YOLODetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Step 3: Language Grounding with CLIP

### Create CLIP Grounding Node

Create `scripts/language_grounding.py`:

```python
#!/usr/bin/env python3
"""
CLIP-based language grounding for object queries
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import String, Int32
from cv_bridge import CvBridge
import clip
import torch
from PIL import Image as PILImage
import numpy as np


class LanguageGroundingNode(Node):
    """Ground natural language queries to detected objects."""

    def __init__(self):
        super().__init__('language_grounding')

        # Load CLIP model
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.model, self.preprocess = clip.load("ViT-B/32", device=self.device)
        self.bridge = CvBridge()

        # State
        self.current_image = None
        self.current_detections = None

        # Subscribers
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )
        self.detection_sub = self.create_subscription(
            Detection2DArray, '/detections', self.detection_callback, 10
        )
        self.query_sub = self.create_subscription(
            String, '/language_query', self.query_callback, 10
        )

        # Publishers
        self.target_pub = self.create_publisher(
            Int32, '/target_object_id', 10
        )

        self.get_logger().info("Language grounding node started")

    def image_callback(self, msg):
        """Store latest image."""
        self.current_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def detection_callback(self, msg):
        """Store latest detections."""
        self.current_detections = msg

    def query_callback(self, msg):
        """Ground language query to object."""
        query = msg.data
        self.get_logger().info(f"Grounding query: '{query}'")

        if self.current_image is None or self.current_detections is None:
            self.get_logger().warn("No image or detections available")
            return

        if len(self.current_detections.detections) == 0:
            self.get_logger().warn("No objects detected")
            return

        # Extract object patches
        patches = []
        for detection in self.current_detections.detections:
            bbox = detection.bbox
            x1 = int(bbox.center.position.x - bbox.size_x / 2)
            y1 = int(bbox.center.position.y - bbox.size_y / 2)
            x2 = int(bbox.center.position.x + bbox.size_x / 2)
            y2 = int(bbox.center.position.y + bbox.size_y / 2)

            # Clamp to image bounds
            h, w = self.current_image.shape[:2]
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(w, x2), min(h, y2)

            patch = self.current_image[y1:y2, x1:x2]
            if patch.size == 0:
                continue

            # Convert to PIL and preprocess
            patch_rgb = cv2.cvtColor(patch, cv2.COLOR_BGR2RGB)
            patch_pil = PILImage.fromarray(patch_rgb)
            patches.append(self.preprocess(patch_pil))

        if len(patches) == 0:
            self.get_logger().warn("No valid patches extracted")
            return

        # Encode images and text
        patches_tensor = torch.stack(patches).to(self.device)
        text_token = clip.tokenize([query]).to(self.device)

        with torch.no_grad():
            image_features = self.model.encode_image(patches_tensor)
            text_features = self.model.encode_text(text_token)

        # Compute similarity
        image_features /= image_features.norm(dim=-1, keepdim=True)
        text_features /= text_features.norm(dim=-1, keepdim=True)
        similarity = (image_features @ text_features.T).squeeze()

        # Find best match
        best_idx = similarity.argmax().item()
        best_score = similarity[best_idx].item()

        self.get_logger().info(f"Best match: object {best_idx} with score {best_score:.3f}")

        # Publish target object ID
        target_msg = Int32()
        target_msg.data = best_idx
        self.target_pub.publish(target_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LanguageGroundingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Step 4: 3D Grasp Planning

### Create Grasp Planner

Create `scripts/grasp_planner.py`:

```python
#!/usr/bin/env python3
"""
Compute 3D grasp poses from 2D detections and depth
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import numpy as np
import cv2
from scipy.spatial.transform import Rotation


class GraspPlanner(Node):
    """Compute 3D grasp poses from visual detections."""

    def __init__(self):
        super().__init__('grasp_planner')

        self.bridge = CvBridge()

        # Camera intrinsics
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        # State
        self.depth_image = None
        self.detections = None
        self.target_object_id = None

        # Subscribers
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth/image_raw', self.depth_callback, 10
        )
        self.detection_sub = self.create_subscription(
            Detection2DArray, '/detections', self.detection_callback, 10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/camera_info', self.camera_info_callback, 10
        )
        self.target_sub = self.create_subscription(
            Int32, '/target_object_id', self.target_callback, 10
        )

        # Publishers
        self.grasp_pose_pub = self.create_publisher(
            PoseStamped, '/grasp_pose', 10
        )

        self.get_logger().info("Grasp planner started")

    def camera_info_callback(self, msg):
        """Store camera intrinsics."""
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    def depth_callback(self, msg):
        """Store latest depth image."""
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')

    def detection_callback(self, msg):
        """Store latest detections."""
        self.detections = msg

    def target_callback(self, msg):
        """Plan grasp for target object."""
        self.target_object_id = msg.data

        if self.detections is None or self.depth_image is None:
            self.get_logger().warn("Missing detections or depth")
            return

        if self.fx is None:
            self.get_logger().warn("Camera intrinsics not received")
            return

        if self.target_object_id >= len(self.detections.detections):
            self.get_logger().error(f"Invalid target ID: {self.target_object_id}")
            return

        # Get target detection
        target_detection = self.detections.detections[self.target_object_id]

        # Compute 3D position
        position_3d = self.compute_3d_position(target_detection.bbox)

        if position_3d is None:
            self.get_logger().error("Failed to compute 3D position")
            return

        # Compute grasp pose
        grasp_pose = self.compute_grasp_pose(position_3d, target_detection.bbox)

        # Publish
        self.grasp_pose_pub.publish(grasp_pose)
        self.get_logger().info(f"Published grasp pose at {position_3d}")

    def compute_3d_position(self, bbox):
        """
        Unproject 2D bbox center to 3D using depth.

        Args:
            bbox: BoundingBox2D message

        Returns:
            np.array: (3,) 3D position [x, y, z] in camera frame
        """
        # Get bbox center pixel
        u = int(bbox.center.position.x)
        v = int(bbox.center.position.y)

        # Clamp to image bounds
        h, w = self.depth_image.shape
        u = np.clip(u, 0, w-1)
        v = np.clip(v, 0, h-1)

        # Get depth
        depth = self.depth_image[v, u]

        if depth == 0 or np.isnan(depth):
            self.get_logger().warn("Invalid depth at bbox center")
            return None

        # Unproject to 3D
        x = (u - self.cx) * depth / self.fx
        y = (v - self.cy) * depth / self.fy
        z = depth

        return np.array([x, y, z])

    def compute_grasp_pose(self, position_3d, bbox):
        """
        Compute 6D grasp pose from 3D position.

        Args:
            position_3d: (3,) 3D position
            bbox: BoundingBox2D (for orientation estimation)

        Returns:
            PoseStamped: Grasp pose
        """
        # Grasp pose
        grasp = PoseStamped()
        grasp.header.frame_id = "camera_link"
        grasp.header.stamp = self.get_clock().now().to_msg()

        # Position (approach from above, 10cm offset)
        grasp.pose.position.x = position_3d[0]
        grasp.pose.position.y = position_3d[1]
        grasp.pose.position.z = position_3d[2] + 0.10  # 10cm above object

        # Orientation (gripper pointing down, aligned with camera)
        # Euler angles: roll=0, pitch=π/2 (pointing down), yaw=0
        rotation = Rotation.from_euler('xyz', [0, np.pi/2, 0])
        quat = rotation.as_quat()  # [x, y, z, w]

        grasp.pose.orientation.x = quat[0]
        grasp.pose.orientation.y = quat[1]
        grasp.pose.orientation.z = quat[2]
        grasp.pose.orientation.w = quat[3]

        return grasp


def main(args=None):
    rclpy.init(args=args)
    node = GraspPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Step 5: Grasp Execution (Action Client)

### Create Manipulation Node

Create `scripts/manipulation_executor.py`:

```python
#!/usr/bin/env python3
"""
Execute grasps via ROS 2 action server
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from control_msgs.action import GripperCommand
from std_msgs.msg import String


class ManipulationExecutor(Node):
    """Execute manipulation tasks."""

    def __init__(self):
        super().__init__('manipulation_executor')

        # Action clients (assuming MoveIt2 or custom controller)
        self.gripper_client = ActionClient(
            self, GripperCommand, '/gripper_controller/gripper_cmd'
        )

        # Subscribers
        self.grasp_pose_sub = self.create_subscription(
            PoseStamped, '/grasp_pose', self.grasp_pose_callback, 10
        )
        self.command_sub = self.create_subscription(
            String, '/manipulation_command', self.command_callback, 10
        )

        # State
        self.current_grasp_pose = None

        self.get_logger().info("Manipulation executor started")

    def grasp_pose_callback(self, msg):
        """Store latest grasp pose."""
        self.current_grasp_pose = msg
        self.get_logger().info(f"Received grasp pose")

    def command_callback(self, msg):
        """Execute manipulation command."""
        command = msg.data

        if command == "execute_grasp":
            self.execute_grasp()
        elif command == "open_gripper":
            self.control_gripper(open=True)
        elif command == "close_gripper":
            self.control_gripper(open=False)

    def execute_grasp(self):
        """Execute full grasp sequence."""
        if self.current_grasp_pose is None:
            self.get_logger().warn("No grasp pose available")
            return

        self.get_logger().info("Executing grasp...")

        # 1. Open gripper
        self.control_gripper(open=True)

        # 2. Move to pre-grasp pose (10cm above object)
        # Note: In real implementation, use MoveIt2 to plan trajectory
        self.get_logger().info("Moving to pre-grasp pose")

        # 3. Move down to grasp pose
        self.get_logger().info("Moving to grasp pose")

        # 4. Close gripper
        self.control_gripper(open=False)

        # 5. Lift object
        self.get_logger().info("Lifting object")

        self.get_logger().info("Grasp execution complete")

    def control_gripper(self, open=True):
        """Open or close gripper."""
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = 0.08 if open else 0.0  # meters
        goal_msg.command.max_effort = 50.0  # Newtons

        self.gripper_client.wait_for_server()
        future = self.gripper_client.send_goal_async(goal_msg)

        rclpy.spin_until_future_complete(self, future)

        if future.result().accepted:
            self.get_logger().info(f"Gripper {'opened' if open else 'closed'}")
        else:
            self.get_logger().error("Gripper command rejected")


def main(args=None):
    rclpy.init(args=args)
    node = ManipulationExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Step 6: Launch File

### Create Launch File

Create `launch/vla_system.launch.py`:

```python
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription([
        # YOLO detector
        Node(
            package='humanoid_vla',
            executable='yolo_detector.py',
            name='yolo_detector',
            output='screen',
            parameters=[{
                'model_name': 'yolov8n.pt',
                'confidence_threshold': 0.5,
                'visualize': True
            }]
        ),

        # Language grounding
        Node(
            package='humanoid_vla',
            executable='language_grounding.py',
            name='language_grounding',
            output='screen'
        ),

        # Grasp planner
        Node(
            package='humanoid_vla',
            executable='grasp_planner.py',
            name='grasp_planner',
            output='screen'
        ),

        # Manipulation executor
        Node(
            package='humanoid_vla',
            executable='manipulation_executor.py',
            name='manipulation_executor',
            output='screen'
        ),

        # RViz2 for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(
                get_package_share_directory('humanoid_vla'),
                'config', 'vla.rviz'
            )]
        ),
    ])
```

## Step 7: Package Configuration

### Update setup.py

```python
from setuptools import setup
from glob import glob
import os

package_name = 'humanoid_vla'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'yolo_detector.py = scripts.yolo_detector:main',
            'language_grounding.py = scripts.language_grounding:main',
            'grasp_planner.py = scripts.grasp_planner:main',
            'manipulation_executor.py = scripts.manipulation_executor:main',
        ],
    },
)
```

### Update package.xml

Add dependencies:

```xml
<depend>sensor_msgs</depend>
<depend>geometry_msgs</depend>
<depend>vision_msgs</depend>
<depend>std_msgs</depend>
<depend>cv_bridge</depend>
<depend>control_msgs</depend>
```

## Step 8: Build and Test

### Build Workspace

```bash
cd ~/humanoid_vla_ws
colcon build --symlink-install
source install/setup.bash
```

### Launch Gazebo Simulation

**Terminal 1 - Gazebo with humanoid:**
```bash
source ~/humanoid_sim_ws/install/setup.bash
ros2 launch humanoid_description spawn_humanoid.launch.py
```

### Launch VLA System

**Terminal 2 - VLA pipeline:**
```bash
source ~/humanoid_vla_ws/install/setup.bash
ros2 launch humanoid_vla vla_system.launch.py
```

### Send Language Command

**Terminal 3 - Language query:**
```bash
# Publish language query
ros2 topic pub --once /language_query std_msgs/msg/String \
  "data: 'red cube'"

# Execute grasp
ros2 topic pub --once /manipulation_command std_msgs/msg/String \
  "data: 'execute_grasp'"
```

**Expected:**
- YOLO detects objects in camera view
- CLIP identifies "red cube" from detections
- Grasp planner computes 3D grasp pose
- Manipulation executor moves gripper to grasp pose
- RViz2 shows detections and grasp pose visualization

## Acceptance Criteria

Your VLA system is complete when:

- [ ] YOLO detections visualized in `/detections/visualization` topic
- [ ] Bounding boxes displayed correctly on camera feed
- [ ] Language query "red cube" correctly identifies target object
- [ ] Grasp pose computed and published to `/grasp_pose`
- [ ] RViz2 displays grasp pose as coordinate frame
- [ ] Pipeline runs at >5 Hz (check with `ros2 topic hz /detections`)
- [ ] Multiple objects distinguished by language queries
- [ ] Depth-to-3D conversion produces valid coordinates

## Troubleshooting

### YOLO not detecting objects

**Solution:** Check camera feed and adjust confidence threshold:
```bash
ros2 param set /yolo_detector confidence_threshold 0.3
```

### CLIP returns wrong object

**Solution:** Ensure detections have sufficient image area. Try more specific queries:
- "red cube" → "small red cube"
- "bottle" → "blue water bottle"

### Grasp pose at (0, 0, 0)

**Solution:** Verify depth image is valid:
```bash
ros2 topic echo /camera/depth/image_raw --once
```

Check camera intrinsics received:
```bash
ros2 topic echo /camera/camera_info --once
```

### Pipeline runs slowly (< 5 Hz)

**Solution:** Use smaller YOLO model or disable visualization:
```bash
ros2 param set /yolo_detector model_name 'yolov8n.pt'  # Nano model
ros2 param set /yolo_detector visualize False
```

## Bonus Challenges

**Challenge 1:** Add object tracking across frames (associate detections over time)
**Challenge 2:** Implement multi-object rearrangement ("move all red objects to the left")
**Challenge 3:** Add collision checking for grasp pose validation
**Challenge 4:** Train custom YOLO model on robot-specific objects

## Key Takeaways

- **YOLO provides real-time object detection from camera feeds**
- **cv_bridge converts ROS Image messages to OpenCV format**
- **CLIP grounds natural language to visual regions**
- **3D grasp poses computed from 2D detections + depth + camera intrinsics**
- **ROS 2 action servers enable asynchronous manipulation control**
- **vision_msgs/Detection2D is standard format for object detections**
- **Complete VLA pipeline: perception → grounding → planning → execution**

## Next Steps

**Congratulations!** You've built a complete Vision-Language-Action system enabling humanoid robots to understand natural language and manipulate objects.

**Continue to [Capstone Project](../../capstone/index.md)** to integrate all four modules:
- ROS 2 communication (Module 1)
- Gazebo simulation (Module 2)
- AI planning and RL (Module 3)
- Vision-language-action (Module 4)

Build an autonomous humanoid robot that navigates environments, recognizes objects, follows natural language instructions, and performs complex manipulation tasks.

---

**Need help?** Review [Lesson 1 (Computer Vision)](lesson-01-computer-vision.md), [Lesson 2 (VLA Models)](lesson-02-vision-language-action.md), or [Lesson 3 (Manipulation)](lesson-03-manipulation-tasks.md).
