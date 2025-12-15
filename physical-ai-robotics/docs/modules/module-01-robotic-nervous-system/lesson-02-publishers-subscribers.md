---
id: lesson-02-publishers-subscribers
title: Lesson 2 - Publishers & Subscribers
sidebar_position: 3
---

# Lesson 2: Publishers & Subscribers

## Introduction

The **publisher-subscriber (pub-sub) pattern** is the most common communication method in ROS 2. It enables decoupled, asynchronous data streaming — perfect for sensor data, state updates, and continuous control signals.

In this lesson, you create your first ROS 2 nodes: a publisher that simulates IMU sensor data and a subscriber that receives joint commands.

## The Publisher-Subscriber Pattern

In the pub-sub pattern:
- **Publishers** send messages to a named topic
- **Subscribers** listen to that topic and receive messages
- Publishers and subscribers are **completely decoupled**

**Key advantages:**
- **One-to-many**: One publisher can send to multiple subscribers
- **Asynchronous**: Publishers do not wait for subscribers
- **Dynamic**: Nodes can join/leave at runtime

### When to Use Pub-Sub

Use topics for:
- Sensor data streaming (camera images, IMU readings)
- State updates (robot joint positions, battery level)
- Continuous commands (velocity setpoints)

## Publisher Example: IMU Sensor

Download the complete code: [imu_publisher.py](/code/module-01/imu_publisher.py)

Key components:

**Create the publisher:**
```python
self.publisher_ = self.create_publisher(Imu, '/imu/data', 10)
self.timer = self.create_timer(0.01, self.timer_callback)
```

**Publish messages:**
```python
def timer_callback(self):
    msg = Imu()
    msg.header.stamp = self.get_clock().now().to_msg()
    msg.linear_acceleration.z = 9.81  # Gravity
    self.publisher_.publish(msg)
```

This publishes IMU data at 100 Hz for humanoid balance control.

## Subscriber Example: Joint Commands

Download the complete code: [joint_subscriber.py](/code/module-01/joint_subscriber.py)

Key components:

**Create the subscription:**
```python
self.subscription = self.create_subscription(
    JointState,
    '/joint_commands',
    self.joint_command_callback,
    10
)
```

**Process messages:**
```python
def joint_command_callback(self, msg):
    for i, joint_name in enumerate(msg.name):
        position = msg.position[i]
        self.get_logger().info(f'Joint: {joint_name}, Pos: {position}')
```

## Try It Yourself

**Terminal 1 (Publisher):**
```bash
python3 imu_publisher.py
```

**Terminal 2 (Subscriber):**
```bash
python3 joint_subscriber.py
```

**Terminal 3 (Monitor topic):**
```bash
ros2 topic echo /imu/data
```

**Terminal 4 (Publish test command):**
```bash
ros2 topic pub /joint_commands sensor_msgs/msg/JointState \
  "{name: ['left_hip'], position: [0.5]}"
```

## Key Takeaways

- Publishers send messages to topics asynchronously
- Subscribers receive messages via callback functions
- Topics enable one-to-many communication
- High-rate streaming (100 Hz) is common for sensors
- Standard message types ensure interoperability

## Next Steps

In **[Lesson 3: Services & Actions](lesson-03-services-actions.md)**, you will learn request-response patterns and long-running tasks.

---

**Estimated time for Lesson 3:** 1.5 hours
