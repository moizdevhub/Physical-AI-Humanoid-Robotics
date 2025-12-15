---
id: lesson-01-ros2-basics
title: Lesson 1 - ROS 2 Basics
sidebar_position: 2
---

# Lesson 1: ROS 2 Basics

## Introduction

**ROS 2 (Robot Operating System 2)** is a flexible middleware framework that serves as the "nervous system" for robots. Just as your nervous system coordinates communication between your brain, sensors (eyes, ears, touch), and muscles, ROS 2 enables different parts of a robot system to communicate and work together seamlessly.

In this lesson, you'll learn the fundamental building blocks of ROS 2 and understand why it's essential for building complex robotic systems, especially humanoid robots with dozens of sensors and actuators that must coordinate in real-time.

## Why ROS 2?

Traditional robotics programming often resulted in monolithic applications where everything runs in a single process. This approach has several problems:

- **No modularity**: Changing one component requires recompiling everything
- **No fault isolation**: If one part crashes, the entire system crashes
- **No reusability**: Code written for one robot cannot easily be used on another
- **No distributed computing**: Cannot leverage multiple computers or cores efficiently

ROS 2 solves these problems by providing:

- **Decentralized architecture**: Independent processes communicate via messages  
- **Language agnostic**: Write nodes in Python, C++, or other languages  
- **Real-time capable**: Meets hard real-time deadlines for safety-critical systems  
- **Cross-platform**: Runs on Linux, Windows, macOS, and embedded systems  
- **Industry-grade middleware**: Built on DDS (Data Distribution Service) standard

## Core Concepts

ROS 2 has three fundamental communication patterns. Understanding these is essential for building any robotic system.

### Nodes

A **node** is an independent executable that performs a specific task. Think of nodes as specialized workers:

- **Sensor nodes**: Read data from hardware (cameras, IMUs, LiDAR)
- **Processing nodes**: Analyze data (object detection, path planning)
- **Control nodes**: Send commands to actuators (motor controllers)
- **Monitoring nodes**: Log data, visualize states, detect failures

**Humanoid robot example:**
- `imu_sensor_node` - Reads acceleration/gyro data from IMU
- `joint_state_node` - Publishes current positions of all joints
- `balance_controller` - Processes IMU data to maintain upright posture
- `gripper_control_node` - Controls finger actuators for grasping

Each node runs independently. If the gripper controller crashes, the balance controller keeps running.

### Topics

A **topic** is a named communication channel for streaming data. Nodes publish messages to topics, and other nodes subscribe to receive those messages.

**Key characteristics:**
- **Asynchronous**: Publishers and subscribers do not wait for each other
- **Many-to-many**: Multiple publishers, multiple subscribers
- **Typed**: Each topic has a specific message type
- **Decoupled**: Publishers do not know who is listening

**Humanoid robot example:**

Topic: `/imu/data`
- Publishers: `imu_sensor_node`
- Subscribers: `balance_controller`, `odometry_estimator`, `logger_node`

Topic: `/joint_states`
- Publishers: `joint_state_node`
- Subscribers: `robot_state_publisher`, `joint_controller`, `visualizer`

### Messages

A **message** is a data structure sent over a topic. ROS 2 provides hundreds of standard message types, and you can define custom ones.

**Common message types for humanoid robots:**

| Message Type | Package | Use Case |
|-------------|---------|----------|
| sensor_msgs/Imu | sensor_msgs | IMU acceleration and gyroscope data |
| sensor_msgs/JointState | sensor_msgs | Joint positions, velocities, efforts |
| sensor_msgs/Image | sensor_msgs | Camera RGB images |
| geometry_msgs/Twist | geometry_msgs | Linear and angular velocity commands |
| std_msgs/String | std_msgs | Simple text messages |

## Humanoid Robot Sensor Data Flow

A humanoid robot must constantly monitor its tilt angle and adjust leg motors to prevent falling. This requires:

1. **IMU sensor** measuring tilt
2. **Balance controller** computing corrective motor commands
3. **Motor controllers** adjusting leg joint positions

### The ROS 2 Solution

```
[IMU Hardware] 
    ↓ reads via I2C/SPI
[imu_sensor_node]
    ↓ publishes sensor_msgs/Imu
/imu/data topic
    ↓ subscribes
[balance_controller_node]
    ↓ computes corrections, publishes
/joint_commands topic
    ↓ subscribes
[motor_controller_node]
    ↓ sends PWM signals
[Leg Motors]
```

**Why this design works:**
- **Modularity**: Can replace IMU sensor without changing balance controller
- **Testability**: Can simulate IMU data without physical hardware
- **Reusability**: Same balance controller works on different humanoid models
- **Debugging**: Can log /imu/data topic to analyze sensor behavior

## ROS 2 Graph Visualization

<!-- <!-- ![ROS 2 graph for humanoid robot sensor integration](/img/modules/module-01/ros2-graph-humanoid.png)
*ROS 2 computation graph showing nodes (rectangles), topics (ovals), and data flow (arrows)* -->

:::tip Viewing Your Own Graph
Once you create ROS 2 nodes, visualize the graph with:
```bash
rqt_graph
```
This tool shows live connections between all running nodes!
:::

## ROS 2 vs. ROS 1: What Changed?

| Feature | ROS 1 | ROS 2 |
|---------|-------|-------|
| Architecture | Centralized (roscore required) | Decentralized (no master) |
| Real-time | Not real-time capable | Real-time support via DDS |
| Security | No built-in security | Encryption, authentication |
| Platforms | Linux only | Linux, Windows, macOS, embedded |
| Middleware | Custom TCP/UDP | DDS (industry standard) |
| Python | Python 2 (deprecated) | Python 3 |
| Lifecycle | No node lifecycle | Managed node lifecycle |

**For this course, we use ROS 2 Humble** (LTS release, supported until 2027).

## Key Takeaways

- ROS 2 is middleware that connects robot components via messages
- Nodes are independent programs that do one job well
- Topics are named channels for streaming data (pub-sub pattern)
- Messages are typed data structures (like sensor readings, commands)
- Humanoid robots use ROS 2 to coordinate sensors, controllers, and actuators
- DDS provides the real-time, distributed communication layer

## Next Steps

Ready to write code? In **[Lesson 2: Publishers & Subscribers](lesson-02-publishers-subscribers.md)**, you will:
- Create your first ROS 2 publisher node (simulate IMU sensor data)
- Create a subscriber node (receive and process messages)
- Run both nodes and observe message flow with ros2 topic echo
- Understand Quality of Service (QoS) profiles for reliable communication

**Before moving on**, make sure you understand:
- What a node is (independent executable)
- What a topic is (communication channel)
- What a message is (typed data structure)
- How sensor data flows from hardware to controllers

---

**Estimated time for Lesson 2:** 1.5 hours (reading + coding exercise)
