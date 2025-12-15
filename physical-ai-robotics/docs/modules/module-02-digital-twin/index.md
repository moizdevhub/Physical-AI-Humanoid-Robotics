---
id: index
title: Module 2 - Digital Twin
sidebar_position: 2
---

# Module 2: Digital Twin

## Overview

A **digital twin** is a virtual replica of a physical robot that enables safe testing, rapid iteration, and scenario generation before deploying to hardware. In this module, you'll master **Gazebo** and **NVIDIA Isaac Sim** to create photorealistic simulations of humanoid robots, complete with physics, sensors, and environments.

## Prerequisites

Before starting this module, you must have:

- **Module 1 completion** (ROS 2 fundamentals, pub-sub patterns, workspace setup)
- **ROS 2 Humble** installed and functional
- **Basic understanding of 3D coordinate systems** (x, y, z axes, rotation)

*Recommended:* Familiarity with XML/URDF syntax (we'll teach this from scratch)

## Learning Objectives

By the end of this module, you will be able to:

- **Understand simulation fundamentals** including physics engines, timesteps, and numerical integration
- **Launch and navigate Gazebo environments** (Classic and Fortress)
- **Create URDF robot models** defining links, joints, and collision geometries
- **Simulate sensors** including cameras, LiDAR, IMUs, and contact sensors
- **Integrate Isaac Sim** for GPU-accelerated physics and photorealistic rendering
- **Connect simulations to ROS 2** via topic bridges for seamless testing

## Lessons

This module consists of three conceptual lessons and one hands-on simulation exercise:

### Lesson 1: Simulation Fundamentals
- Why simulate? (safety, cost, speed of iteration)
- Physics engines (ODE, Bullet, PhysX)
- Sensor models and noise characteristics
- Sim-to-real transfer challenges
- Gazebo vs. Isaac Sim vs. Unity (when to use each)

### Lesson 2: Gazebo Basics
- Installing Gazebo Classic and Gazebo Fortress
- Navigating the GUI (camera controls, model insertion)
- World files (.world SDF format)
- Spawning models and adjusting physics parameters
- ROS 2 Gazebo bridge (ros_gz_bridge)

### Lesson 3: URDF Robot Models
- Unified Robot Description Format (URDF) structure
- Links, joints, and kinematic chains
- Visual vs. collision geometries (why separate?)
- Inertial properties and center of mass
- Xacro macros for modular robot descriptions
- Visualizing URDF in RViz2

### Exercise: Humanoid Simulation in Gazebo
**Hands-On Project:** Launch a simulated humanoid robot in Gazebo and control it via ROS 2 topics.

**You will:**
1. Download a sample humanoid URDF model (e.g., NAO, Pepper, or simplified biped)
2. Spawn the robot in a Gazebo world with obstacles
3. Publish joint commands from ROS 2 nodes (reusing Module 1 publisher code)
4. Subscribe to simulated camera and IMU sensor data
5. Visualize sensor streams in RViz2

**Acceptance Criteria:**
- Gazebo window shows humanoid robot standing in environment
- `ros2 topic echo /joint_states` displays real-time joint positions
- Camera feed visible in RViz2 image viewer
- Robot responds to published joint velocity commands

**What you'll learn:** How to create a complete digital twin that mirrors physical robot behavior, enabling you to test control algorithms safely before hardware deployment.

## Exercise Preview

You'll launch a humanoid robot in Gazebo and control its joints:

```bash
# Terminal 1: Launch Gazebo with humanoid model
ros2 launch humanoid_gazebo spawn_robot.launch.py

# Terminal 2: Publish joint commands
ros2 topic pub /joint_commands sensor_msgs/msg/JointState \
  "{name: ['left_hip', 'right_hip'], position: [0.5, 0.5]}"

# Terminal 3: Visualize in RViz2
rviz2 -d config/humanoid.rviz
```

You'll see the robot's legs move in real-time as you adjust joint angles, demonstrating the ROS 2 ↔ Gazebo integration.

## Tools & Setup

You'll use:

- **Gazebo Classic 11** or **Gazebo Fortress** (both compatible with ROS 2 Humble)
- **RViz2** (ROS 2 visualization tool)
- **URDF/Xacro** tools (`urdf_parser`, `xacro`)
- **NVIDIA Isaac Sim 2023.1** (optional, for GPU-accelerated advanced scenarios)

Installation instructions provided in Lesson 2.

## Estimated Time

- **Lessons 1-3:** 5-7 hours (reading + URDF modeling practice)
- **Exercise:** 4-5 hours (Gazebo setup + ROS 2 integration)
- **Total:** ~10-12 hours over Weeks 4-6

---

**Ready to start?** Begin with **Lesson 1: Simulation Fundamentals** to understand physics engines and sensor models.

**Have Gazebo installed?** Jump to **Lesson 3: URDF Robot Models** to start building robot descriptions.
