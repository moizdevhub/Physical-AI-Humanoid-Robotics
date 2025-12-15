---
id: lesson-03-manipulation-tasks
title: Lesson 3 - Manipulation Tasks
sidebar_position: 4
---

# Lesson 3: Manipulation Tasks

## Introduction

**Manipulation** — the ability to interact with and rearrange objects — is fundamental to useful humanoid robots. Unlike navigation, manipulation requires precise control, contact-rich interactions, and reasoning about object affordances. Language-conditioned manipulation enables robots to follow natural instructions like "place the cup on the shelf" or "hand me the red tool."

In this lesson, you'll learn grasp planning, language-conditioned object rearrangement, Task and Motion Planning (TAMP), error recovery strategies, and sim-to-real transfer for manipulation.

## Grasp Planning

### Grasp Taxonomy

**Grasps categorized by contact type:**

**1. Parallel Jaw Grasp**
- Two-finger gripper (most common on robots)
- Approach direction perpendicular to object surface
- Good for: boxes, bottles, cylinders

**2. Power Grasp**
- Whole hand encloses object
- High force, low precision
- Good for: large objects, heavy items

**3. Precision Grasp**
- Fingertip control
- High precision, low force
- Good for: small objects, delicate items

**4. Pinch Grasp**
- Thumb + finger
- Good for: flat objects, papers

**For humanoid robots:** Parallel jaw grasp is most common due to hardware simplicity.

### Grasp Pose Representation

**6D grasp pose:**
```python
grasp_pose = {
    'position': (x, y, z),          # 3D position in world frame
    'orientation': (qx, qy, qz, qw), # Quaternion orientation
    'gripper_width': w,              # Opening width (meters)
    'approach_direction': (dx, dy, dz), # Pre-grasp approach vector
}
```

**Constraints:**
- **Collision-free:** Gripper doesn't collide with object/environment
- **Force closure:** Grasp resists external forces
- **Reachability:** Robot arm can reach pose

### Analytic Grasp Planning

**Compute grasps from object geometry.**

**For box-shaped objects:**
```python
def generate_box_grasps(bbox_min, bbox_max, num_grasps=8):
    """
    Generate grasps around a bounding box.

    Args:
        bbox_min: (3,) minimum corner
        bbox_max: (3,) maximum corner
        num_grasps: number of candidate grasps

    Returns:
        grasp_poses: List of (position, orientation, width)
    """
    grasps = []

    # Object center and dimensions
    center = (bbox_min + bbox_max) / 2
    size = bbox_max - bbox_min

    # Generate grasps from 4 sides (top-down approach)
    angles = np.linspace(0, 2*np.pi, num_grasps, endpoint=False)

    for angle in angles:
        # Approach from angle
        approach = np.array([np.cos(angle), np.sin(angle), 0])

        # Grasp position (above object)
        position = center + approach * (np.linalg.norm(size[:2])/2 + 0.05)
        position[2] = bbox_max[2] + 0.05  # 5cm above object

        # Orientation (gripper pointing down, rotated around Z)
        # Represented as quaternion
        orientation = euler_to_quaternion(0, np.pi, angle)

        # Gripper width (slightly larger than object)
        width = min(size[0], size[1]) + 0.02

        grasps.append({
            'position': position,
            'orientation': orientation,
            'width': width,
            'approach': approach
        })

    return grasps
```

**Collision checking:**
```python
def is_collision_free(grasp_pose, scene_point_cloud):
    """
    Check if gripper at grasp_pose collides with scene.

    Args:
        grasp_pose: Grasp 6D pose
        scene_point_cloud: (N, 3) 3D points

    Returns:
        bool: True if collision-free
    """
    # Get gripper geometry at grasp pose
    gripper_points = transform_gripper_mesh(grasp_pose)

    # Check distance to scene points
    distances = cdist(gripper_points, scene_point_cloud)
    min_distance = np.min(distances)

    # Collision if any point closer than threshold
    return min_distance > 0.01  # 1cm safety margin
```

### Learning-Based Grasp Detection

**GraspNet: Deep learning for grasp prediction.**

**Architecture:**
- Input: RGB-D image (640×480×4)
- Output: Grasp heatmap + gripper orientation field

**Training:**
```python
import torch
import torch.nn as nn

class GraspNet(nn.Module):
    """CNN for grasp detection."""

    def __init__(self):
        super().__init__()

        # Encoder (ResNet backbone)
        self.encoder = torchvision.models.resnet18(pretrained=True)
        self.encoder.conv1 = nn.Conv2d(4, 64, 7, 2, 3)  # 4 channels (RGBD)

        # Decoder heads
        self.grasp_quality_head = nn.Conv2d(512, 1, 1)  # Grasp quality per pixel
        self.angle_head = nn.Conv2d(512, 1, 1)  # Gripper angle per pixel
        self.width_head = nn.Conv2d(512, 1, 1)  # Gripper width per pixel

    def forward(self, rgbd_image):
        # Encode
        features = self.encoder(rgbd_image)  # (batch, 512, H/32, W/32)

        # Decode
        quality = torch.sigmoid(self.grasp_quality_head(features))
        angle = torch.tanh(self.angle_head(features)) * np.pi  # [-π, π]
        width = torch.sigmoid(self.width_head(features)) * 0.1  # [0, 0.1m]

        return quality, angle, width


# Inference
def detect_grasps(model, rgbd_image):
    """Detect grasp candidates in image."""
    with torch.no_grad():
        quality, angle, width = model(rgbd_image)

    # Find top-K grasp locations
    quality_map = quality.squeeze().cpu().numpy()
    top_k_indices = np.argpartition(quality_map.flatten(), -10)[-10:]

    grasps = []
    for idx in top_k_indices:
        row, col = np.unravel_index(idx, quality_map.shape)

        grasp = {
            'pixel': (col, row),
            'quality': quality_map[row, col],
            'angle': angle[0, 0, row, col].item(),
            'width': width[0, 0, row, col].item()
        }
        grasps.append(grasp)

    return grasps
```

**Advantages over analytic:**
- Handles complex geometries
- Learns from data (no hand-engineering)
- Generalizes to novel objects

## Language-Conditioned Manipulation

### CLIP for Object Grounding

**CLIP (Contrastive Language-Image Pretraining) grounds language to vision.**

**Use case:** "Pick up the red mug" → Identify "red mug" in image.

```python
import clip
import torch

class LanguageGrounding:
    """Ground language queries to image regions using CLIP."""

    def __init__(self):
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.model, self.preprocess = clip.load("ViT-B/32", device=self.device)

    def ground_object(self, image, object_descriptions, bboxes):
        """
        Find object matching text description.

        Args:
            image: (H, W, 3) RGB image
            object_descriptions: List of strings (e.g., ["red mug", "blue bottle"])
            bboxes: List of (x1, y1, x2, y2) bounding boxes

        Returns:
            best_idx: Index of best-matching bounding box
            score: Similarity score
        """
        # Extract image patches for each bbox
        patches = []
        for x1, y1, x2, y2 in bboxes:
            patch = image[int(y1):int(y2), int(x1):int(x2)]
            patches.append(self.preprocess(Image.fromarray(patch)))

        patches_tensor = torch.stack(patches).to(self.device)

        # Encode images and text
        with torch.no_grad():
            image_features = self.model.encode_image(patches_tensor)
            text_tokens = clip.tokenize(object_descriptions).to(self.device)
            text_features = self.model.encode_text(text_tokens)

        # Compute similarity
        image_features /= image_features.norm(dim=-1, keepdim=True)
        text_features /= text_features.norm(dim=-1, keepdim=True)
        similarity = (image_features @ text_features.T)  # (num_patches, num_descriptions)

        # Find best match
        best_idx = similarity.argmax(dim=0)[0].item()  # For first description
        best_score = similarity[best_idx, 0].item()

        return best_idx, best_score


# Example usage
grounding = LanguageGrounding()

# Detected objects
bboxes = [(100, 100, 200, 250), (300, 150, 400, 300)]  # YOLO detections

# Language query
query = ["red mug"]

# Find matching object
best_idx, score = grounding.ground_object(image, query, bboxes)
target_bbox = bboxes[best_idx]

print(f"Target object at bbox {target_bbox} with score {score:.2f}")
```

### Pick-and-Place Pipeline

**Complete language-conditioned pick-and-place:**

```python
class PickAndPlacePipeline:
    """Language-conditioned pick-and-place."""

    def __init__(self):
        self.detector = YOLODetectionNode()
        self.grounding = LanguageGrounding()
        self.grasp_planner = GraspPlanner()
        self.motion_planner = MoveItInterface()

    def execute(self, instruction):
        """
        Execute pick-and-place from language instruction.

        Example: "pick up the red cube and place it on the blue plate"
        """
        # Parse instruction
        pick_object, place_object = self.parse_instruction(instruction)
        # Returns: ("red cube", "blue plate")

        # 1. Detect objects
        image = self.get_camera_image()
        detections = self.detector.detect(image)

        # 2. Ground target object
        bboxes = [d.bbox for d in detections]
        pick_idx, _ = self.grounding.ground_object(image, [pick_object], bboxes)
        place_idx, _ = self.grounding.ground_object(image, [place_object], bboxes)

        pick_bbox = bboxes[pick_idx]
        place_bbox = bboxes[place_idx]

        # 3. Compute 3D positions from depth
        depth_image = self.get_depth_image()
        pick_position_3d = self.unproject(pick_bbox, depth_image)
        place_position_3d = self.unproject(place_bbox, depth_image)

        # 4. Plan grasp
        grasp_pose = self.grasp_planner.plan(pick_position_3d, pick_bbox)

        # 5. Execute pick
        self.motion_planner.move_to_pose(grasp_pose)
        self.motion_planner.close_gripper()
        self.motion_planner.lift(height=0.2)

        # 6. Execute place
        place_pose = self.compute_place_pose(place_position_3d)
        self.motion_planner.move_to_pose(place_pose)
        self.motion_planner.open_gripper()

        # 7. Retract
        self.motion_planner.retract()

        return True

    def unproject(self, bbox, depth_image):
        """Convert 2D bbox + depth to 3D position."""
        x1, y1, x2, y2 = bbox
        center_x = int((x1 + x2) / 2)
        center_y = int((y1 + y2) / 2)

        # Get depth at center
        depth = depth_image[center_y, center_x]

        # Unproject using camera intrinsics
        X = (center_x - cx) * depth / fx
        Y = (center_y - cy) * depth / fy
        Z = depth

        return np.array([X, Y, Z])
```

## Task and Motion Planning (TAMP)

**TAMP combines high-level task planning with low-level motion planning.**

### The TAMP Problem

**Example task:** "Clear the table"

**High-level plan:**
```
1. Pick up mug
2. Place mug in sink
3. Pick up plate
4. Place plate in cabinet
5. Pick up fork
6. Place fork in drawer
```

**Challenge:** Task plan must be **feasible** (robot can execute motions).

### PDDLStream

**Planning Domain Definition Language with Streams.**

**Domain:** Objects, predicates, actions

```lisp
(define (domain kitchen)
  (:predicates
    (On ?obj ?surface)
    (Clear ?surface)
    (Holding ?obj)
    (HandEmpty))

  (:action pick
    :parameters (?obj ?surface)
    :precondition (and (On ?obj ?surface) (Clear ?obj) (HandEmpty))
    :effect (and (Holding ?obj) (not (On ?obj ?surface)) (not (HandEmpty)) (Clear ?surface)))

  (:action place
    :parameters (?obj ?surface)
    :precondition (and (Holding ?obj) (Clear ?surface))
    :effect (and (On ?obj ?surface) (not (Holding ?obj)) (HandEmpty) (not (Clear ?surface))))
)
```

**Problem:** Initial state → Goal state

```lisp
(define (problem clear-table)
  (:init
    (On mug table)
    (On plate table)
    (Clear mug)
    (Clear plate)
    (HandEmpty))

  (:goal
    (and (On mug sink) (On plate cabinet)))
)
```

**Planner output:**
```
1. pick(mug, table)
2. place(mug, sink)
3. pick(plate, table)
4. place(plate, cabinet)
```

### Integrating Motion Planning

**Stream:** Samples geometric parameters (grasp poses, placements).

```python
def sample_grasp(obj):
    """Sample grasp pose for object."""
    grasps = generate_box_grasps(obj.bbox_min, obj.bbox_max)
    for grasp in grasps:
        if is_collision_free(grasp, scene):
            yield grasp  # Generator yields feasible grasps

def sample_placement(obj, surface):
    """Sample placement pose on surface."""
    for _ in range(100):
        x = np.random.uniform(surface.x_min, surface.x_max)
        y = np.random.uniform(surface.y_min, surface.y_max)
        z = surface.height
        pose = (x, y, z)
        if is_stable(obj, pose) and not collides(obj, pose):
            yield pose
```

**TAMP algorithm:**
1. Task planner generates symbolic plan
2. For each action, sample geometric parameters (streams)
3. Check motion feasibility with motion planner
4. If feasible: execute. If not: replan or resample.

## Error Recovery

### Common Manipulation Failures

**1. Grasp failure (object slips)**
- **Detect:** Force/torque sensor readings below threshold
- **Recovery:** Regrasp with different orientation

**2. Collision during motion**
- **Detect:** Unexpected contact, joint torque spike
- **Recovery:** Retreat to safe pose, replan collision-free path

**3. Object not found**
- **Detect:** YOLO detection confidence < threshold
- **Recovery:** Active search (move head/camera to scan environment)

**4. Place location occupied**
- **Detect:** Depth sensor shows obstacle at target location
- **Recovery:** Find alternative placement location

### Error Recovery Framework

```python
class RobustManipulation:
    """Manipulation with error recovery."""

    def __init__(self):
        self.max_retries = 3

    def robust_pick(self, target_object):
        """Pick with retry logic."""
        for attempt in range(self.max_retries):
            try:
                # Attempt pick
                grasp_pose = self.plan_grasp(target_object)
                self.execute_grasp(grasp_pose)

                # Verify grasp success
                if self.is_grasping():
                    self.logger.info("Grasp successful")
                    return True
                else:
                    self.logger.warn(f"Grasp failed, attempt {attempt+1}/{self.max_retries}")
                    # Try different grasp
                    continue

            except CollisionError as e:
                self.logger.error(f"Collision during grasp: {e}")
                self.retreat_to_safe_pose()
                # Replan with collision avoidance

            except ObjectNotFoundError as e:
                self.logger.error(f"Object not found: {e}")
                self.active_search(target_object)

        # All retries failed
        self.logger.error("Pick failed after all retries")
        return False

    def is_grasping(self):
        """Check if object is grasped."""
        # Option 1: Force/torque sensor
        force = self.get_gripper_force()
        return force > 1.0  # Newton

        # Option 2: Gripper width
        width = self.get_gripper_width()
        return width < self.max_gripper_width - 0.01  # Object present

    def retreat_to_safe_pose(self):
        """Move to pre-defined safe configuration."""
        safe_joints = [0, -np.pi/4, 0, -np.pi/2, 0, np.pi/3, 0]
        self.motion_planner.move_to_joint_positions(safe_joints)
```

## Sim-to-Real Transfer for Manipulation

### Domain Randomization

**Randomize simulation parameters to bridge sim-to-real gap.**

**Visual randomization:**
```python
def randomize_visual_scene():
    """Randomize visual appearance in Gazebo."""
    # Object textures
    object_textures = load_texture_library()
    for obj in scene.objects:
        obj.set_texture(random.choice(object_textures))

    # Lighting
    light_intensity = np.random.uniform(0.5, 2.0)
    light_position = np.random.uniform(-5, 5, size=3)
    scene.set_light(intensity=light_intensity, position=light_position)

    # Camera noise
    camera.add_gaussian_noise(std=0.01)
```

**Physics randomization:**
```python
def randomize_physics():
    """Randomize object properties."""
    for obj in scene.objects:
        # Mass
        nominal_mass = obj.get_mass()
        obj.set_mass(np.random.uniform(0.8 * nominal_mass, 1.2 * nominal_mass))

        # Friction
        obj.set_friction(np.random.uniform(0.3, 1.2))

        # Center of mass offset
        com_offset = np.random.normal(0, 0.01, size=3)
        obj.set_center_of_mass(obj.get_center_of_mass() + com_offset)
```

### System Identification

**Measure real-world parameters, update simulation.**

```python
def calibrate_simulation():
    """Match simulation to real robot."""
    # 1. Measure gripper force-displacement curve
    real_forces, real_displacements = measure_gripper_on_real_robot()

    # 2. Fit simulation parameters
    def objective(params):
        sim_forces = simulate_gripper(params, real_displacements)
        return np.mean((sim_forces - real_forces)**2)

    optimal_params = scipy.optimize.minimize(objective, initial_guess)

    # 3. Update simulation
    set_simulation_parameters(optimal_params)
```

### Real-World Fine-Tuning

**Few-shot adaptation on real robot.**

**Process:**
1. Train policy in simulation (1M steps)
2. Collect 100-500 real-world demos
3. Fine-tune policy on real data (10K steps)

```python
# Fine-tuning recipe
pretrained_model = load_model("sim_trained_policy.pth")

# Freeze early layers (keep visual features)
for param in pretrained_model.vision_encoder.parameters():
    param.requires_grad = False

# Train only action decoder
optimizer = torch.optim.Adam(pretrained_model.action_decoder.parameters(), lr=1e-5)

for episode in real_world_dataset:
    loss = compute_bc_loss(pretrained_model, episode)
    optimizer.zero_grad()
    loss.backward()
    optimizer.step()
```

## Key Takeaways

- **Grasp planning: analytic (geometry-based) or learned (GraspNet)**
- **Parallel jaw grasp most common for humanoid robots**
- **CLIP enables language grounding (map "red mug" to image region)**
- **Pick-and-place pipeline: detect → ground → grasp → place**
- **TAMP combines symbolic task planning with geometric motion planning**
- **Error recovery essential: retry with different grasps, replan motions**
- **Sim-to-real: domain randomization + system ID + real-world fine-tuning**
- **Force/torque sensors critical for detecting grasp success**

## Next Steps

In the **[Exercise: Object Detection and Language-Conditioned Grasping](exercise-object-detection-grasping.md)**, you will:
- Stream camera images from Gazebo to ROS 2
- Run YOLO object detection on camera feed
- Integrate CLIP for language grounding
- Compute 3D grasp poses from 2D detections and depth
- Execute grasps via ROS 2 action server
- Visualize detections in RViz2 with bounding boxes

**Before starting the exercise**, make sure you can:
- Generate grasp candidates from bounding boxes
- Use CLIP to match language queries to image regions
- Understand the pick-and-place pipeline steps
- Recognize common manipulation failure modes and recovery strategies

---

**Estimated time for Exercise:** 6-7 hours (YOLO integration + grasp planning + ROS 2 actions)
