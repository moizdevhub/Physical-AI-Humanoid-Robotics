---
id: lesson-03-urdf-models
title: Lesson 3 - URDF Robot Models
sidebar_position: 4
---

# Lesson 3: URDF Robot Models

## Introduction

**URDF (Unified Robot Description Format)** is the standard XML format for describing robot kinematics, dynamics, and visual appearance in ROS 2. Every robot in Gazebo needs a URDF to define its structure.

In this lesson, you'll learn to create URDF files for humanoid robots, define links and joints, set physical properties, and use Xacro for modular descriptions.

## URDF Basics

### What is URDF?

URDF is an XML format that specifies:
- **Links**: Rigid body parts (torso, arm, leg)
- **Joints**: Connections between links (revolute, prismatic)
- **Visual**: 3D meshes for rendering
- **Collision**: Simplified geometry for physics
- **Inertial**: Mass, center of mass, inertia tensor

### Minimal URDF Example

```xml
<?xml version="1.0"?>
<robot name="simple_robot">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.6 0.4 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.6 0.4 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0"
               iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>
</robot>
```

## Links

A **link** is a rigid body element of the robot.

### Visual Geometry

Defines how the link looks (rendering).

**Primitive shapes:**
```xml
<visual>
  <geometry>
    <!-- Box -->
    <box size="1.0 0.5 0.3"/>  <!-- Length x Width x Height -->

    <!-- Cylinder -->
    <cylinder radius="0.1" length="1.0"/>

    <!-- Sphere -->
    <sphere radius="0.2"/>
  </geometry>
</visual>
```

**Mesh files (STL, DAE, OBJ):**
```xml
<visual>
  <geometry>
    <mesh filename="package://my_robot/meshes/torso.stl" scale="1.0 1.0 1.0"/>
  </geometry>
</visual>
```

### Collision Geometry

Defines physics interaction (should be simpler than visual).

**Why separate from visual?**
- Complex meshes slow down physics
- Simplified collision = faster simulation
- Often use boxes/cylinders instead of detailed meshes

```xml
<collision>
  <geometry>
    <box size="0.6 0.4 0.2"/>  <!-- Simpler than visual mesh -->
  </geometry>
</collision>
```

### Inertial Properties

Defines mass distribution for physics.

**Components:**
- **mass**: Total mass in kilograms
- **origin**: Center of mass position
- **inertia**: Moment of inertia tensor (6 values)

```xml
<inertial>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
  <mass value="5.4"/>
  <inertia ixx="0.1" ixy="0.0" ixz="0.0"
           iyy="0.1" iyz="0.0" izz="0.05"/>
</inertial>
```

**Calculating inertia for simple shapes:**

**Box (dimensions: x, y, z):**
```
Ixx = (1/12) * mass * (y² + z²)
Iyy = (1/12) * mass * (x² + z²)
Izz = (1/12) * mass * (x² + y²)
```

**Cylinder (radius: r, length: h):**
```
Ixx = Iyy = (1/12) * mass * (3r² + h²)
Izz = (1/2) * mass * r²
```

**Sphere (radius: r):**
```
Ixx = Iyy = Izz = (2/5) * mass * r²
```

## Joints

A **joint** connects two links and defines allowed motion.

### Joint Types

**1. Revolute (Rotation with limits)**
```xml
<joint name="knee_joint" type="revolute">
  <parent link="thigh"/>
  <child link="shin"/>
  <origin xyz="0 0 -0.4" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>  <!-- Rotation around Y-axis -->
  <limit lower="0" upper="2.4" effort="100" velocity="1.0"/>
</joint>
```
- **Use:** Robot joints with stops (elbow, knee)
- **Limits:** Min/max angle (radians)

**2. Continuous (Unlimited rotation)**
```xml
<joint name="wheel_joint" type="continuous">
  <parent link="base"/>
  <child link="wheel"/>
  <axis xyz="0 1 0"/>
</joint>
```
- **Use:** Wheels, rotating sensors
- **No limits:** Can spin forever

**3. Prismatic (Linear motion)**
```xml
<joint name="elevator_joint" type="prismatic">
  <parent link="base"/>
  <child link="platform"/>
  <axis xyz="0 0 1"/>  <!-- Move along Z-axis -->
  <limit lower="0" upper="1.0" effort="500" velocity="0.5"/>
</joint>
```
- **Use:** Telescoping arms, elevators
- **Limits:** Min/max position (meters)

**4. Fixed (No motion)**
```xml
<joint name="camera_mount" type="fixed">
  <parent link="head"/>
  <child link="camera"/>
  <origin xyz="0.1 0 0" rpy="0 0 0"/>
</joint>
```
- **Use:** Sensors rigidly attached to links
- **No motion:** Link is fixed relative to parent

### Humanoid Joint Example

**Hip joint (3-DOF using 3 revolute joints):**
```xml
<!-- Hip roll (abduction/adduction) -->
<joint name="left_hip_roll" type="revolute">
  <parent link="torso"/>
  <child link="left_hip"/>
  <axis xyz="1 0 0"/>
  <limit lower="-0.5" upper="0.5" effort="100" velocity="1.0"/>
</joint>

<!-- Hip pitch (flexion/extension) -->
<joint name="left_hip_pitch" type="revolute">
  <parent link="left_hip"/>
  <child link="left_thigh"/>
  <axis xyz="0 1 0"/>
  <limit lower="-1.5" upper="1.5" effort="150" velocity="1.0"/>
</joint>

<!-- Hip yaw (rotation) -->
<joint name="left_hip_yaw" type="revolute">
  <parent link="left_thigh"/>
  <child link="left_shin"/>
  <axis xyz="0 0 1"/>
  <limit lower="-0.3" upper="0.3" effort="80" velocity="1.0"/>
</joint>
```

## Xacro: Programmable URDF

**Xacro** (XML Macros) extends URDF with:
- Variables and constants
- Mathematical expressions
- Macros (reusable components)
- Conditional logic

### Why Use Xacro?

**Problem:** Humanoid robot with 2 identical legs
- URDF: Copy-paste code for left/right (error-prone)
- Xacro: Define leg once, instantiate twice (maintainable)

### Xacro Basics

**Variables:**
```xml
<xacro:property name="leg_length" value="0.8"/>
<xacro:property name="leg_mass" value="2.5"/>
```

**Math expressions:**
```xml
<mass value="${leg_mass / 2}"/>  <!-- Half mass -->
<origin xyz="0 0 ${leg_length}" rpy="0 0 0"/>
```

**Macros:**
```xml
<xacro:macro name="leg" params="prefix reflect">
  <link name="${prefix}_thigh">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.4"/>
      </geometry>
    </visual>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.1" iyy="0.1" izz="0.05"
               ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>

  <joint name="${prefix}_hip" type="revolute">
    <parent link="torso"/>
    <child link="${prefix}_thigh"/>
    <origin xyz="0 ${reflect * 0.2} 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.5" upper="1.5" effort="100" velocity="1.0"/>
  </joint>
</xacro:macro>
```

**Instantiate macro:**
```xml
<xacro:leg prefix="left" reflect="1"/>
<xacro:leg prefix="right" reflect="-1"/>
```

### Complete Xacro Example

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid">
  <!-- Constants -->
  <xacro:property name="torso_mass" value="10.0"/>
  <xacro:property name="leg_mass" value="3.0"/>

  <!-- Torso -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.4 0.3 0.6"/>
      </geometry>
    </visual>
    <inertial>
      <mass value="${torso_mass}"/>
      <inertia ixx="0.5" iyy="0.5" izz="0.3"
               ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>

  <!-- Leg macro -->
  <xacro:macro name="leg" params="prefix y_offset">
    <link name="${prefix}_thigh">
      <visual>
        <geometry>
          <cylinder radius="0.08" length="0.4"/>
        </geometry>
      </visual>
      <inertial>
        <mass value="${leg_mass}"/>
        <inertia ixx="0.1" iyy="0.1" izz="0.05"
                 ixy="0" ixz="0" iyz="0"/>
      </inertial>
    </link>

    <joint name="${prefix}_hip" type="revolute">
      <parent link="base_link"/>
      <child link="${prefix}_thigh"/>
      <origin xyz="0 ${y_offset} -0.3" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit lower="-1.5" upper="1.5" effort="100" velocity="1.0"/>
    </joint>
  </xacro:macro>

  <!-- Instantiate legs -->
  <xacro:leg prefix="left" y_offset="0.15"/>
  <xacro:leg prefix="right" y_offset="-0.15"/>
</robot>
```

**Convert Xacro to URDF:**
```bash
xacro humanoid.xacro > humanoid.urdf
```

## Gazebo Integration

### Gazebo-specific Tags

**Gazebo material (color):**
```xml
<gazebo reference="base_link">
  <material>Gazebo/Blue</material>
</gazebo>
```

**Gazebo physics:**
```xml
<gazebo reference="base_link">
  <mu1>0.8</mu1>  <!-- Friction coefficient 1 -->
  <mu2>0.8</mu2>  <!-- Friction coefficient 2 -->
  <kp>1000000</kp>  <!-- Contact stiffness -->
  <kd>1</kd>  <!-- Contact damping -->
</gazebo>
```

**Gazebo sensors:**
```xml
<gazebo reference="camera_link">
  <sensor type="camera" name="camera1">
    <update_rate>30.0</update_rate>
    <camera>
      <horizontal_fov>1.3962634</horizontal_fov>
      <image>
        <width>800</width>
        <height>600</height>
      </image>
    </camera>
  </sensor>
</gazebo>
```

## Spawning URDF in Gazebo

### Method 1: Launch File

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': open('humanoid.urdf').read()}]
        ),
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-name', 'my_robot', '-file', 'humanoid.urdf'],
            output='screen'
        ),
    ])
```

### Method 2: Command Line

```bash
# Start Gazebo
gz sim empty_world.sdf

# Spawn robot
ros2 run ros_gz_sim create -name my_robot -file humanoid.urdf
```

## Key Takeaways

- **URDF defines robot structure: links, joints, properties**
- **Links have visual, collision, and inertial components**
- **Visual can be detailed meshes, collision should be simple**
- **Joints connect links: revolute, continuous, prismatic, fixed**
- **Inertial properties (mass, inertia) are critical for physics**
- **Xacro adds variables, math, and macros to URDF**
- **Gazebo tags enable materials, sensors, and physics tuning**
- **Spawn robots via launch files or command line**

## Next Steps

**Congratulations!** You've completed Module 2: Digital Twin.

You now understand:
- Why simulation is essential for robotics
- How to navigate Gazebo and spawn models
- How to create URDF robot descriptions
- How to connect Gazebo to ROS 2

**Continue to [Module 3: AI-Robot Brain](../module-03-ai-robot-brain/index.md)** to learn:
- Path planning algorithms (A*, RRT)
- Reinforcement learning for robot control
- Behavior trees for task orchestration
- Nav2 navigation stack integration

---

**Estimated time for Module 3:** 12-14 hours over 3 weeks
