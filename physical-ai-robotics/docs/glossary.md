---
id: glossary
title: Glossary
sidebar_position: 6
---

# Glossary

Comprehensive terminology for Physical AI and Humanoid Robotics. Terms are organized alphabetically with cross-references.

---

## A

### Action {#action}
In ROS 2, a communication pattern for **long-running tasks** that provides:
- Goal specification (e.g., "navigate to position X")
- Periodic feedback (e.g., "50% complete")
- Result confirmation (success/failure)
- Preemption capability (cancel mid-execution)

Contrast with [Services](#service) (synchronous, no feedback) and [Topics](#topic) (asynchronous, no request-response).

### Action Space {#action-space}
The set of all possible robot actions (e.g., joint velocities, gripper open/close). In VLA models, continuous action predictions must be discretized or normalized to match robot capabilities.

### AI (Artificial Intelligence) {#ai}
The simulation of human intelligence in machines, enabling learning, reasoning, and decision-making. In robotics, AI powers perception (vision), cognition (planning), and action (control).

### A* (A-star) {#astar}
A graph-based pathfinding algorithm that uses heuristics to efficiently find optimal paths. Commonly used for global planning in robot navigation.

---

## B

### Behavior Tree {#behavior-tree}
A hierarchical task decomposition structure for robot control. Nodes include:
- **Sequence**: Execute children in order (fails if any child fails)
- **Selector**: Try children until one succeeds
- **Parallel**: Run children concurrently
- **Decorator**: Modify child behavior (e.g., retry, timeout)

Contrast with finite state machines (FSMs).

### BLIP (Bootstrapping Language-Image Pretraining) {#blip}
A vision-language model that jointly trains on image-text pairs for tasks like image captioning and visual question answering.

---

## C

### CLIP (Contrastive Language-Image Pretraining) {#clip}
An OpenAI vision-language model that learns to match images with text descriptions. Used in robotics for language grounding (e.g., "find the red cube" → bounding box).

### Collision Geometry {#collision-geometry}
Simplified 3D shapes (boxes, cylinders, meshes) used by physics engines to detect collisions. Often simpler than visual geometry for computational efficiency.

### Colcon {#colcon}
The **col**lective **con**struction tool for building ROS 2 workspaces. Replaces `catkin_make` from ROS 1.

---

## D

### DDS (Data Distribution Service) {#dds}
The middleware layer used by ROS 2 for real-time, decentralized communication. Provides QoS (Quality of Service) profiles for reliability and latency tuning.

### Digital Twin {#digital-twin}
A virtual replica of a physical robot used for simulation, testing, and development. Enables safe experimentation before hardware deployment.

### DQN (Deep Q-Network) {#dqn}
A value-based reinforcement learning algorithm that uses deep neural networks to approximate Q-values (expected future rewards).

---

## E

### Embodied AI {#embodied-ai}
Artificial intelligence systems that are **physically grounded** in robot bodies, enabling interaction with the real world. Contrast with purely digital AI (e.g., chatbots).

### Embodied Intelligence {#embodied-intelligence}
The integration of perception, cognition, and action in physical agents. Requires coordinating vision, language understanding, planning, and motor control.

---

## G

### Gazebo {#gazebo}
An open-source 3D robotics simulator supporting realistic physics, sensor simulation, and ROS 2 integration via `ros_gz_bridge`. Versions include Gazebo Classic and Gazebo Fortress/Garden.

---

## H

### Humanoid Robot {#humanoid-robot}
A robot with **human-like body structure**: head, torso, two arms, two legs. Examples: Atlas (Boston Dynamics), NAO (SoftBank), Pepper, Digit (Agility Robotics).

---

## I

### IMU (Inertial Measurement Unit) {#imu}
A sensor measuring acceleration and angular velocity. Used for robot balance, odometry, and state estimation.

### Isaac Sim {#isaac-sim}
NVIDIA's robotics simulation platform built on Omniverse, featuring GPU-accelerated physics (PhysX), photorealistic rendering, and ROS 2 integration.

---

## N

### Nav2 (Navigation 2) {#nav2}
The ROS 2 navigation stack for mobile robots, featuring global planning, local planning, costmap generation, and behavior trees.

### Node {#node}
In ROS 2, an independent executable that performs computation and communicates via Topics, Services, or Actions.

---

## P

### Physical AI {#physical-ai}
Artificial intelligence designed to control **physical embodiments** (robots) in real-world environments. Emphasizes perception, planning, and action execution.

### Publisher {#publisher}
A ROS 2 node that sends messages on a Topic.

---

## R

### ROS 2 (Robot Operating System 2) {#ros2}
A flexible framework for writing robot software with decentralized communication (DDS), real-time support, and cross-platform compatibility.

### RT-1 (Robotics Transformer 1) {#rt1}
Google's VLA model trained on robot manipulation data using Vision Transformer architecture.

### RT-2 (Robotics Transformer 2) {#rt2}
Successor to RT-1 integrating vision-language models for improved language grounding and generalization.

### RViz2 {#rviz2}
ROS 2 visualization tool for displaying robot models, sensor data, planned paths, and transforms.

---

## S

### Service {#service}
In ROS 2, a synchronous request-response communication pattern. Contrast with Topics (asynchronous) and Actions (long-running).

### Subscriber {#subscriber}
A ROS 2 node that receives messages from a Topic.

---

## T

### Topic {#topic}
In ROS 2, a named bus over which nodes exchange messages via publisher-subscriber pattern. Asynchronous, many-to-many communication.

---

## U

### URDF (Unified Robot Description Format) {#urdf}
XML format for describing robot models in ROS, defining links, joints, visual/collision geometries, and inertial properties.

---

## V

### VLA (Vision-Language-Action) {#vla}
Models that integrate vision (process images), language (parse commands), and action (generate robot controls). Examples: RT-1, RT-2, Octo, OpenVLA.

### Visuomotor Control {#visuomotor-control}
Direct mapping from visual inputs (camera images) to motor commands (joint velocities). End-to-end learning approach.

---

## Y

### YOLO (You Only Look Once) {#yolo}
A real-time object detection model (versions: YOLOv5, YOLOv8) used in robotics for visual perception.

---

*This glossary will be expanded as new terms are introduced throughout the course. Deep-link anchors enable cross-referencing from module content.*
