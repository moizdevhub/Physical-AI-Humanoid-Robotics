---
id: exercise-humanoid-gazebo
title: Exercise - Humanoid Robot in Gazebo
sidebar_position: 5
---

# Exercise: Spawn a Humanoid Robot in Gazebo

## Overview

In this hands-on exercise, you'll create a simplified humanoid robot URDF, spawn it in Gazebo, and control it via ROS 2 topics. This demonstrates the complete digital twin workflow from model creation to simulation.

By the end, you'll have a walking bipedal robot simulation connected to ROS 2.

**Estimated time:** 2-3 hours

## Learning Objectives

After completing this exercise, you will be able to:

✅ Create a URDF file with multiple links and joints
✅ Use Xacro macros for symmetric robot parts
✅ Spawn custom robots in Gazebo
✅ Publish joint commands via ROS 2
✅ Visualize robot state in RViz2
✅ Bridge Gazebo sensors to ROS 2 topics

## Prerequisites

**Required:**
- ROS 2 Humble with Gazebo Fortress installed
- Module 1 completion (ROS 2 basics)
- Lessons 1-3 of Module 2

**Verify installation:**
```bash
gz sim --version  # Should show Gazebo Sim 7.x
ros2 --version    # Should show ROS 2 Humble
```

## Step 1: Create Workspace

```bash
mkdir -p ~/humanoid_sim_ws/src
cd ~/humanoid_sim_ws/src
ros2 pkg create --build-type ament_python humanoid_description
cd humanoid_description
mkdir urdf launch meshes
```

## Step 2: Create Simple Humanoid URDF

Create `urdf/simple_humanoid.xacro`:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="simple_humanoid">
  <!-- Constants -->
  <xacro:property name="torso_mass" value="5.0"/>
  <xacro:property name="leg_mass" value="1.5"/>
  <xacro:property name="foot_mass" value="0.5"/>

  <!-- Torso (base link) -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="${torso_mass}"/>
      <inertia ixx="0.2" iyy="0.2" izz="0.1"
               ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>

  <!-- Leg macro -->
  <xacro:macro name="leg" params="prefix y_offset">
    <!-- Thigh -->
    <link name="${prefix}_thigh">
      <visual>
        <geometry>
          <cylinder radius="0.05" length="0.3"/>
        </geometry>
        <material name="gray">
          <color rgba="0.5 0.5 0.5 1"/>
        </material>
      </visual>
      <collision>
        <geometry>
          <cylinder radius="0.05" length="0.3"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="${leg_mass}"/>
        <inertia ixx="0.01" iyy="0.01" izz="0.005"
                 ixy="0" ixz="0" iyz="0"/>
      </inertial>
    </link>

    <!-- Hip joint -->
    <joint name="${prefix}_hip" type="revolute">
      <parent link="base_link"/>
      <child link="${prefix}_thigh"/>
      <origin xyz="0 ${y_offset} -0.25" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit lower="-1.57" upper="1.57" effort="50" velocity="1.0"/>
    </joint>

    <!-- Shin -->
    <link name="${prefix}_shin">
      <visual>
        <geometry>
          <cylinder radius="0.04" length="0.3"/>
        </geometry>
        <material name="gray">
          <color rgba="0.5 0.5 0.5 1"/>
        </material>
      </visual>
      <collision>
        <geometry>
          <cylinder radius="0.04" length="0.3"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="${leg_mass}"/>
        <inertia ixx="0.01" iyy="0.01" izz="0.005"
                 ixy="0" ixz="0" iyz="0"/>
      </inertial>
    </link>

    <!-- Knee joint -->
    <joint name="${prefix}_knee" type="revolute">
      <parent link="${prefix}_thigh"/>
      <child link="${prefix}_shin"/>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit lower="0" upper="2.4" effort="50" velocity="1.0"/>
    </joint>

    <!-- Foot -->
    <link name="${prefix}_foot">
      <visual>
        <geometry>
          <box size="0.15 0.08 0.03"/>
        </geometry>
        <material name="black">
          <color rgba="0.1 0.1 0.1 1"/>
        </material>
      </visual>
      <collision>
        <geometry>
          <box size="0.15 0.08 0.03"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="${foot_mass}"/>
        <inertia ixx="0.001" iyy="0.001" izz="0.001"
                 ixy="0" ixz="0" iyz="0"/>
      </inertial>
    </link>

    <!-- Ankle joint -->
    <joint name="${prefix}_ankle" type="revolute">
      <parent link="${prefix}_shin"/>
      <child link="${prefix}_foot"/>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit lower="-0.7" upper="0.7" effort="20" velocity="1.0"/>
    </joint>

    <!-- Gazebo friction for foot -->
    <gazebo reference="${prefix}_foot">
      <mu1>1.0</mu1>
      <mu2>1.0</mu2>
      <kp>1000000</kp>
      <kd>1</kd>
    </gazebo>
  </xacro:macro>

  <!-- Instantiate legs -->
  <xacro:leg prefix="left" y_offset="0.1"/>
  <xacro:leg prefix="right" y_offset="-0.1"/>

  <!-- Gazebo materials -->
  <gazebo reference="base_link">
    <material>Gazebo/Blue</material>
  </gazebo>
</robot>
```

**Convert to URDF:**
```bash
cd ~/humanoid_sim_ws/src/humanoid_description
xacro urdf/simple_humanoid.xacro > urdf/simple_humanoid.urdf
```

## Step 3: Create Launch File

Create `launch/spawn_humanoid.launch.py`:

```python
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('humanoid_description')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'simple_humanoid.urdf')

    # Read URDF file
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    return LaunchDescription([
        # Start Gazebo
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', 'empty.sdf'],
            output='screen'
        ),

        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}],
            output='screen'
        ),

        # Spawn robot in Gazebo
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-name', 'humanoid',
                      '-topic', '/robot_description',
                      '-x', '0', '-y', '0', '-z', '0.5'],
            output='screen'
        ),

        # Bridge joint states
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model'],
            output='screen'
        ),
    ])
```

## Step 4: Update Package Configuration

Edit `setup.py`:

```python
from setuptools import setup
import os
from glob import glob

package_name = 'humanoid_description'

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
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Humanoid robot description for Gazebo',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [],
    },
)
```

Edit `package.xml` to add dependencies:

```xml
<depend>ros_gz_sim</depend>
<depend>ros_gz_bridge</depend>
<depend>robot_state_publisher</depend>
<depend>xacro</depend>
```

## Step 5: Build and Launch

```bash
cd ~/humanoid_sim_ws
colcon build --symlink-install
source install/setup.bash
```

**Launch the simulation:**
```bash
ros2 launch humanoid_description spawn_humanoid.launch.py
```

**Expected:** Gazebo opens with humanoid robot standing.

## Step 6: Control the Robot

**Terminal 2 - List joints:**
```bash
ros2 topic list | grep joint
```

**Terminal 2 - Publish joint commands:**
```bash
ros2 topic pub /joint_commands sensor_msgs/msg/JointState \
  "{name: ['left_hip', 'left_knee'], position: [0.5, 1.0]}" --once
```

**Expected:** Robot leg moves to specified positions.

## Step 7: Visualize in RViz2

**Terminal 3 - Launch RViz2:**
```bash
ros2 run rviz2 rviz2
```

**Configure RViz2:**
1. Set Fixed Frame: `base_link`
2. Add → RobotModel
   - Description Topic: `/robot_description`
3. Add → TF
   - Shows coordinate frames
4. Add → JointState (if available)

**Expected:** Robot model appears in RViz2 matching Gazebo.

## Acceptance Criteria

Your simulation is complete when:

- [ ] Gazebo shows humanoid robot with torso and 2 legs
- [ ] Robot stands without falling over
- [ ] `ros2 topic list` shows `/robot_description` and `/joint_states`
- [ ] Publishing to `/joint_commands` moves robot joints
- [ ] RViz2 displays robot model with TF frames
- [ ] `colcon build` succeeds without errors
- [ ] Robot has proper friction (feet don't slide)

## Troubleshooting

### Robot falls through ground

**Solution:** Add ground plane to world file or increase contact stiffness:
```xml
<gazebo reference="left_foot">
  <kp>1000000</kp>
  <kd>1</kd>
</gazebo>
```

### Joints don't move

**Solution:** Check joint limits and effort values:
```xml
<limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
```

### RViz2 shows "No transform from..."

**Solution:** Verify robot_state_publisher is running:
```bash
ros2 node list | grep robot_state
```

### Gazebo crashes on launch

**Solution:** Check URDF syntax:
```bash
check_urdf urdf/simple_humanoid.urdf
```

## Bonus Challenges

**Challenge 1:** Add arms to the humanoid (shoulders, elbows)
**Challenge 2:** Add an IMU sensor to the torso
**Challenge 3:** Implement a simple balance controller
**Challenge 4:** Create a walking gait by publishing joint trajectories

## Key Takeaways

- URDF + Xacro = modular robot descriptions
- Gazebo spawns robots from URDF via ros_gz_sim
- robot_state_publisher broadcasts TF transforms
- Joint commands control simulated actuators
- RViz2 visualizes robot state independent of Gazebo
- Proper inertia and friction are critical for stability

## Next Steps

**Congratulations!** You've created and simulated a custom humanoid robot.

**Continue to [Module 3: AI-Robot Brain](../module-03-ai-robot-brain/index.md)** to:
- Implement path planning for navigation
- Train RL policies for balance control
- Use behavior trees for task sequencing
- Integrate Nav2 for autonomous navigation

---

**Need help?** Review [Lesson 3 (URDF Models)](lesson-03-urdf-models.md) for URDF syntax or [Lesson 2 (Gazebo Basics)](lesson-02-gazebo-basics.md) for Gazebo troubleshooting.
