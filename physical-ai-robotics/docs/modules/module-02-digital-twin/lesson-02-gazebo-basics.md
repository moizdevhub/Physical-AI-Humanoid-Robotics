---
id: lesson-02-gazebo-basics
title: Lesson 2 - Gazebo Basics
sidebar_position: 3
---

# Lesson 2: Gazebo Basics

## Introduction

**Gazebo** is the most popular open-source robot simulator with deep ROS 2 integration. It provides realistic physics, sensors, and rendering for testing humanoid robots before hardware deployment.

In this lesson, you'll install Gazebo, learn to navigate the interface, spawn models, and connect simulations to ROS 2.

## Gazebo Versions

### Gazebo Classic (Legacy)

- **Version:** Gazebo 11 (final release)
- **ROS 2:** Compatible via gazebo_ros_pkgs
- **Status:** Maintenance mode, no new features
- **Use when:** Existing projects, stability needed

### Gazebo (New Generation)

- **Formerly:** Ignition Gazebo
- **Versions:** Fortress (LTS), Garden, Harmonic
- **ROS 2:** Native integration via ros_gz
- **Status:** Active development
- **Use when:** New projects, modern features

**For this course:** We'll use **Gazebo Fortress** (LTS release compatible with ROS 2 Humble).

## Installation

### Install Gazebo Fortress

**Ubuntu 22.04 with ROS 2 Humble:**

```bash
sudo apt update
sudo apt install ros-humble-ros-gz
```

This installs:
- Gazebo Fortress simulator
- ros_gz bridge (ROS 2 ↔ Gazebo communication)
- Example worlds and models

**Verify installation:**
```bash
gz sim --version
# Output: Gazebo Sim, version 7.x.x
```

### Install Additional Tools

```bash
# RViz2 for visualization
sudo apt install ros-humble-rviz2

# Robot state publisher
sudo apt install ros-humble-robot-state-publisher

# Joint state publisher
sudo apt install ros-humble-joint-state-publisher-gui
```

## Gazebo GUI Overview

### Launch Gazebo

```bash
gz sim
```

This opens the Gazebo GUI with an empty world.

### Interface Components

**1. Scene View (Center)**
- 3D rendering of simulation world
- Camera controls for navigation
- Model selection and manipulation

**2. World Tree (Left Panel)**
- Hierarchical list of models in scene
- Expand to see links, joints, sensors
- Right-click for options

**3. Component Inspector (Right Panel)**
- Properties of selected model
- Position, rotation, physics settings
- Sensor configurations

**4. Toolbar (Top)**
- Play/Pause simulation
- Step simulation (single time step)
- Real-time factor display

**5. Insert Panel**
- Model library browser
- Drag-and-drop model insertion
- Search for robots, objects, terrains

## Camera Navigation

### Mouse Controls

**Orbit (Rotate around point):**
- Middle-click + drag
- Rotates view around center point

**Pan (Move view):**
- Shift + Middle-click + drag
- Moves camera laterally

**Zoom:**
- Scroll wheel
- Move closer/farther from center

**First-Person (WASD):**
- Right-click + WASD keys
- W: Forward, S: Backward
- A: Left, D: Right
- Q: Down, E: Up

### View Presets

**Top-down view:**
- Press `2` key
- Good for navigation planning

**Side view:**
- Press `3` key
- Good for height adjustments

**Front view:**
- Press `1` key

**Reset camera:**
- Press `R` key

## Spawning Models

### Built-in Models

**1. Click Insert Tab**
- Browse model library
- Categories: Robots, Furniture, Terrain

**2. Select Model**
- Example: Select "Ground Plane"
- Model appears attached to cursor

**3. Place Model**
- Click in scene to place
- Model spawns at click location

**Common models:**
- `Ground Plane` - Flat surface
- `Sun` - Directional light source
- `Box` - Simple cube (1m sides)
- `Sphere` - Simple sphere (1m diameter)
- `Cylinder` - Simple cylinder

### Custom Models

**Download model from Fuel (Gazebo model repository):**

```bash
# Example: Download a humanoid model
wget https://fuel.gazebosim.org/1.0/OpenRobotics/models/Humanoid
```

**Load in Gazebo:**
- Set environment variable:
```bash
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:~/models
```

- Restart Gazebo
- Model appears in Insert panel

## World Files

### What is a World File?

A `.sdf` (Simulation Description Format) file that defines:
- Models in the scene
- Lighting (sun, ambient, shadows)
- Physics settings (gravity, timestep)
- Plugin configurations

### Example World File

Create `~/empty_world.sdf`:

```xml
<?xml version="1.0"?>
<sdf version="1.8">
  <world name="empty_world">
    <!-- Physics settings -->
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <!-- Lighting -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>1.0 1.0 1.0 1</diffuse>
    </light>

    <!-- Ground plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

**Load world file:**
```bash
gz sim ~/empty_world.sdf
```

## ROS 2 Integration

### The ros_gz Bridge

The bridge connects ROS 2 topics to Gazebo topics:

**ROS 2 → Gazebo:** Send velocity commands to simulated robot
**Gazebo → ROS 2:** Receive sensor data (camera, IMU, LiDAR)

### Launch Bridge

**Terminal 1 - Start Gazebo:**
```bash
gz sim empty_world.sdf
```

**Terminal 2 - Launch bridge:**
```bash
ros2 run ros_gz_bridge parameter_bridge /camera@sensor_msgs/msg/Image@gz.msgs.Image
```

This bridges:
- Gazebo topic: `/camera` (type: gz.msgs.Image)
- ROS 2 topic: `/camera` (type: sensor_msgs/msg/Image)

**Terminal 3 - Verify ROS 2 topic:**
```bash
ros2 topic list
# Should show /camera
```

### Common Bridge Mappings

| ROS 2 Message | Gazebo Message | Use Case |
|---------------|----------------|----------|
| sensor_msgs/Image | gz.msgs.Image | Camera feed |
| sensor_msgs/Imu | gz.msgs.IMU | IMU data |
| sensor_msgs/LaserScan | gz.msgs.LaserScan | LiDAR |
| geometry_msgs/Twist | gz.msgs.Twist | Velocity commands |
| sensor_msgs/JointState | gz.msgs.Model | Joint positions |

## Visualizing in RViz2

### Launch RViz2

```bash
ros2 run rviz2 rviz2
```

### Add Displays

**1. Add Camera Display:**
- Click "Add" button
- Select "Image"
- Set Topic: `/camera`
- Camera feed appears

**2. Add TF Display:**
- Click "Add"
- Select "TF"
- Shows coordinate frames

**3. Add Robot Model:**
- Click "Add"
- Select "RobotModel"
- Set topic to `/robot_description`

## Physics Configuration

### Adjusting Timestep

Smaller timestep = More accurate but slower

**Edit world file:**
```xml
<physics name="fast" type="ignored">
  <max_step_size>0.01</max_step_size>  <!-- 10ms -->
  <real_time_factor>1.0</real_time_factor>
</physics>
```

**Typical values:**
- High accuracy: 0.001s (1ms)
- Standard: 0.005s (5ms)
- Fast prototyping: 0.01s (10ms)

### Gravity

**Earth gravity (default):**
```xml
<gravity>0 0 -9.81</gravity>
```

**Moon gravity (for testing):**
```xml
<gravity>0 0 -1.62</gravity>
```

**Zero gravity (space):**
```xml
<gravity>0 0 0</gravity>
```

## Plugins

Gazebo plugins extend functionality.

### System Plugins

**Physics plugin:**
```xml
<plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
```

**Sensor plugin:**
```xml
<plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"/>
```

**Scene broadcaster:**
```xml
<plugin filename="gz-sim-scene-broadcaster-system"
        name="gz::sim::systems::SceneBroadcaster"/>
```

### Model Plugins

**Differential drive (mobile robot):**
```xml
<plugin filename="gz-sim-diff-drive-system"
        name="gz::sim::systems::DiffDrive">
  <left_joint>left_wheel_joint</left_joint>
  <right_joint>right_wheel_joint</right_joint>
</plugin>
```

## Key Takeaways

- **Gazebo Fortress is the modern version for ROS 2 Humble**
- **GUI has scene view, world tree, inspector, toolbar**
- **Camera controls: Middle-click orbit, Shift+middle pan, scroll zoom**
- **World files (.sdf) define scenes, physics, lighting**
- **ros_gz_bridge connects ROS 2 topics to Gazebo topics**
- **RViz2 visualizes sensor data from Gazebo**
- **Physics timestep affects accuracy and speed**
- **Plugins extend Gazebo functionality**

## Next Steps

In **[Lesson 3: URDF Robot Models](lesson-03-urdf-models.md)**, you will:
- Understand URDF format for robot descriptions
- Create links and joints for humanoid robots
- Define collision and visual geometries
- Set inertial properties (mass, inertia)
- Use Xacro for modular robot descriptions
- Spawn custom robots in Gazebo

**Before moving on**, make sure you can:
- Launch Gazebo and navigate the scene
- Insert models from the library
- Create and load a world file
- Bridge Gazebo topics to ROS 2
- Visualize data in RViz2

---

**Estimated time for Lesson 3:** 2 hours (URDF modeling)
