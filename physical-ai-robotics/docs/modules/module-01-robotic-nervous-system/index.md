---
id: index
title: Module 1 - Robotic Nervous System
sidebar_position: 1
---

# Module 1: Robotic Nervous System

## Overview

Just as the human nervous system coordinates communication between the brain, sensors, and muscles, **ROS 2 (Robot Operating System 2)** serves as the "nervous system" for humanoid robots. This module teaches you to build distributed communication architectures that enable real-time coordination between perception, decision-making, and action.

## Prerequisites

Before starting this module, you should have:

- **Python 3.8+** programming skills (functions, classes, modules, basic OOP)
- **Command-line familiarity** (navigating directories, running scripts, installing packages)
- **Ubuntu 22.04 LTS** environment (native, VM, or WSL2)

*No prior robotics or ROS experience required* — we start from fundamentals.

## Learning Objectives

By the end of this module, you will be able to:

- **Understand ROS 2 architecture** including nodes, topics, services, actions, and the DDS middleware layer
- **Implement publisher-subscriber patterns** for asynchronous sensor data streaming and actuator control
- **Create service clients and servers** for synchronous request-response interactions (e.g., "Is path clear?")
- **Use action servers** for long-running tasks with feedback and cancellation (e.g., "Navigate to goal")
- **Organize ROS 2 workspaces** with packages, launch files, and parameter configurations
- **Debug ROS 2 systems** using command-line tools (ros2 topic, ros2 node, rqt_graph)

## Lessons

This module consists of three conceptual lessons and one hands-on exercise:

### Lesson 1: ROS 2 Basics
- ROS 2 vs. ROS 1 (why the redesign?)
- Nodes as independent executables
- DDS (Data Distribution Service) middleware
- Setting up a ROS 2 Humble workspace
- Creating your first Python node

### Lesson 2: Publishers & Subscribers  
- Topic-based asynchronous communication
- Message types (std_msgs, geometry_msgs, sensor_msgs)
- Publishing sensor data (e.g., simulated IMU readings)
- Subscribing to actuator commands
- Quality of Service (QoS) profiles for reliability

### Lesson 3: Services & Actions
- Services for synchronous request-response patterns
- Creating custom service definitions
- Actions for long-running tasks with feedback
- Action client-server architecture
- When to use topics vs. services vs. actions

### Exercise: ROS 2 Communication Workspace
**Hands-On Project:** Build a complete ROS 2 workspace demonstrating all three communication patterns.

**You will:**
1. Create a publisher node that simulates humanoid robot joint positions
2. Create a subscriber node that displays joint states in real-time
3. Implement a service to query current robot status
4. Build an action server for a simulated "move arm" task with progress feedback

**Acceptance Criteria:**
- `ros2 topic list` shows your custom topics
- `ros2 node list` displays all running nodes
- Service call returns robot status successfully
- Action client sends goal and receives feedback updates

**What you'll learn:** By building this workspace, you'll understand how ROS 2 nodes communicate, how to structure packages, and how to debug distributed systems using ROS 2 tools.

## Exercise Preview

In the hands-on exercise, you'll set up a ROS 2 workspace from scratch and implement a **simulated humanoid joint controller**:

```python
# Example: Publishing joint positions
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointPublisher(Node):
    def __init__(self):
        super().__init__('joint_publisher')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
        
    def timer_callback(self):
        msg = JointState()
        msg.name = ['left_shoulder', 'right_shoulder', 'left_elbow']
        msg.position = [0.5, 0.5, 1.2]  # Radians
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing joint states')
```

This exercise prepares you for Module 2, where you'll connect these ROS 2 nodes to a full humanoid robot simulation in Gazebo.

## Tools & Setup

You'll use:

- **ROS 2 Humble** (LTS release, Ubuntu 22.04)
- **Python 3.10** (comes with Ubuntu 22.04)
- **colcon** (ROS 2 build tool)
- **rqt** (graphical introspection tools)

Installation instructions are provided in Lesson 1.

## Estimated Time

- **Lessons 1-3:** 4-6 hours (reading + concept review)
- **Exercise:** 3-4 hours (workspace setup + coding)
- **Total:** ~8-10 hours over Week 1-3

---

**Ready to start?** Begin with **Lesson 1: ROS 2 Basics** (coming soon) to set up your development environment.

**Have ROS 2 installed already?** Jump to **Lesson 2: Publishers & Subscribers** to start coding.
