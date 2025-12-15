---
id: exercise-ros2-workspace
title: Exercise - Build a ROS 2 Workspace
sidebar_position: 5
---

# Exercise: Build a ROS 2 Communication Workspace

## Overview

In this hands-on exercise, you'll build a complete ROS 2 workspace demonstrating **all three communication patterns** learned in Module 1:

- **Topics** (Publisher-Subscriber) - Sensor data streaming
- **Services** (Request-Response) - Status queries
- **Actions** (Goal-Feedback-Result) - Long-running tasks

By the end, you'll have a working humanoid robot control simulation running multiple nodes and communication patterns.

**Estimated time:** 2-3 hours

## Learning Objectives

After completing this exercise, you will be able to:

✅ Create a ROS 2 workspace with custom packages
✅ Write publisher and subscriber nodes for sensor data
✅ Implement a service server and client
✅ Build an action server with feedback
✅ Test nodes using ROS 2 command-line tools
✅ Visualize node graphs with rqt_graph

## Prerequisites

**Required:**
- ROS 2 Humble installed (see [Installation Guide](https://docs.ros.org/en/humble/Installation.html))
- Python 3.8+
- Basic terminal/command-line skills
- Completion of Lessons 1-3

**Verify ROS 2 installation:**
```bash
ros2 --version
# Should output: ros2 cli version 0.18.x
```

## Project Structure

You will create this workspace structure:

```
humanoid_control_ws/
├── src/
│   ├── humanoid_sensors/
│   │   ├── package.xml
│   │   ├── setup.py
│   │   ├── humanoid_sensors/
│   │   │   ├── __init__.py
│   │   │   ├── imu_publisher.py      # Topic publisher
│   │   │   └── joint_subscriber.py   # Topic subscriber
│   │
│   ├── humanoid_services/
│   │   ├── package.xml
│   │   ├── setup.py
│   │   ├── humanoid_services/
│   │   │   ├── __init__.py
│   │   │   ├── status_service.py     # Service server
│   │   │   └── status_client.py      # Service client
│   │
│   └── humanoid_actions/
│       ├── package.xml
│       ├── setup.py
│       ├── humanoid_actions/
│       │   ├── __init__.py
│       │   ├── move_arm_server.py    # Action server
│       │   └── move_arm_client.py    # Action client
│
├── install/
├── build/
└── log/
```

## Step 1: Create Workspace

### 1.1 Create workspace directory

```bash
mkdir -p ~/humanoid_control_ws/src
cd ~/humanoid_control_ws/src
```

### 1.2 Source ROS 2 environment

```bash
source /opt/ros/humble/setup.bash
```

**Add to ~/.bashrc for automatic sourcing:**
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

## Step 2: Create Package for Topics (Sensors)

### 2.1 Create package

```bash
cd ~/humanoid_control_ws/src
ros2 pkg create --build-type ament_python humanoid_sensors \
  --dependencies rclpy sensor_msgs std_msgs
```

### 2.2 Create IMU publisher node

Create `~/humanoid_control_ws/src/humanoid_sensors/humanoid_sensors/imu_publisher.py`:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
import math
import time


class ImuPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        self.publisher_ = self.create_publisher(Imu, '/imu/data', 10)
        self.timer = self.create_timer(0.01, self.timer_callback)  # 100 Hz
        self.sequence = 0
        self.start_time = time.time()
        self.get_logger().info('IMU Publisher started - Publishing at 100 Hz')

    def timer_callback(self):
        msg = Imu()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        elapsed = time.time() - self.start_time
        tilt = 0.05 * math.sin(elapsed)

        msg.orientation.x = tilt
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        msg.orientation.w = math.sqrt(1.0 - tilt**2)

        msg.angular_velocity.x = 0.05 * math.cos(elapsed)
        msg.linear_acceleration.z = 9.81

        self.publisher_.publish(msg)
        self.sequence += 1

        if self.sequence % 100 == 0:
            self.get_logger().info(f'Published IMU #{self.sequence}')


def main(args=None):
    rclpy.init(args=args)
    node = ImuPublisher()
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

### 2.3 Create joint subscriber node

Create `~/humanoid_control_ws/src/humanoid_sensors/humanoid_sensors/joint_subscriber.py`:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointSubscriber(Node):
    def __init__(self):
        super().__init__('joint_subscriber')
        self.subscription = self.create_subscription(
            JointState, '/joint_commands', self.callback, 10)
        self.get_logger().info('Joint Subscriber started')

    def callback(self, msg):
        self.get_logger().info('Received joint command:')
        for i, name in enumerate(msg.name):
            pos = msg.position[i] if i < len(msg.position) else 0.0
            self.get_logger().info(f'  {name}: {pos:.3f} rad')


def main(args=None):
    rclpy.init(args=args)
    node = JointSubscriber()
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

### 2.4 Update setup.py

Edit `~/humanoid_control_ws/src/humanoid_sensors/setup.py`:

```python
from setuptools import setup

package_name = 'humanoid_sensors'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Humanoid robot sensor nodes',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'imu_publisher = humanoid_sensors.imu_publisher:main',
            'joint_subscriber = humanoid_sensors.joint_subscriber:main',
        ],
    },
)
```

## Step 3: Create Package for Services

### 3.1 Create package

```bash
cd ~/humanoid_control_ws/src
ros2 pkg create --build-type ament_python humanoid_services \
  --dependencies rclpy std_srvs
```

### 3.2 Create status service server

Create `~/humanoid_control_ws/src/humanoid_services/humanoid_services/status_service.py`:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class StatusService(Node):
    def __init__(self):
        super().__init__('status_service')
        self.srv = self.create_service(
            Trigger, 'get_robot_status', self.status_callback)
        self.get_logger().info('Status Service ready')

    def status_callback(self, request, response):
        response.success = True
        response.message = 'Robot status: OPERATIONAL - Balance OK, Battery 85%'
        self.get_logger().info('Status request received')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = StatusService()
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

### 3.3 Create status client

Create `~/humanoid_control_ws/src/humanoid_services/humanoid_services/status_client.py`:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class StatusClient(Node):
    def __init__(self):
        super().__init__('status_client')
        self.client = self.create_client(Trigger, 'get_robot_status')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')

    def send_request(self):
        request = Trigger.Request()
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


def main(args=None):
    rclpy.init(args=args)
    client = StatusClient()
    response = client.send_request()
    client.get_logger().info(f'Response: {response.message}')
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 3.4 Update setup.py

Edit `~/humanoid_control_ws/src/humanoid_services/setup.py` to add entry points:

```python
entry_points={
    'console_scripts': [
        'status_service = humanoid_services.status_service:main',
        'status_client = humanoid_services.status_client:main',
    ],
},
```

## Step 4: Build the Workspace

### 4.1 Build all packages

```bash
cd ~/humanoid_control_ws
colcon build --symlink-install
```

**Expected output:**
```
Starting >>> humanoid_sensors
Starting >>> humanoid_services
Finished <<< humanoid_sensors [2.3s]
Finished <<< humanoid_services [2.4s]

Summary: 2 packages finished
```

### 4.2 Source the workspace

```bash
source ~/humanoid_control_ws/install/setup.bash
```

**Add to ~/.bashrc:**
```bash
echo "source ~/humanoid_control_ws/install/setup.bash" >> ~/.bashrc
```

## Step 5: Test Your Workspace

### 5.1 Test Topic Communication

**Terminal 1 - Start IMU publisher:**
```bash
ros2 run humanoid_sensors imu_publisher
```

**Terminal 2 - Monitor topic:**
```bash
ros2 topic echo /imu/data
```

**Terminal 3 - Start joint subscriber:**
```bash
ros2 run humanoid_sensors joint_subscriber
```

**Terminal 4 - Publish joint command:**
```bash
ros2 topic pub /joint_commands sensor_msgs/msg/JointState \
  "{name: ['left_hip', 'right_hip'], position: [0.5, -0.5]}"
```

**Expected:** Subscriber logs received joint commands.

### 5.2 Test Service Communication

**Terminal 1 - Start service server:**
```bash
ros2 run humanoid_services status_service
```

**Terminal 2 - Call service:**
```bash
ros2 service call /get_robot_status std_srvs/srv/Trigger
```

**Expected output:**
```
success: True
message: 'Robot status: OPERATIONAL - Balance OK, Battery 85%'
```

**Or use the client:**
```bash
ros2 run humanoid_services status_client
```

### 5.3 Verify with ROS 2 Tools

**List all active nodes:**
```bash
ros2 node list
```

**List all topics:**
```bash
ros2 topic list
```

**List all services:**
```bash
ros2 service list
```

**Visualize node graph:**
```bash
rqt_graph
```

## Acceptance Criteria

Your workspace is complete when you can demonstrate:

- [ ] `ros2 topic list` shows `/imu/data` and `/joint_commands`
- [ ] `ros2 node list` displays all running nodes
- [ ] IMU publisher logs messages every second
- [ ] Joint subscriber receives and logs commands
- [ ] Service call returns robot status
- [ ] `rqt_graph` shows node connections
- [ ] `colcon build` succeeds without errors

## Troubleshooting

### Issue: "Package not found"

**Solution:**
```bash
source ~/humanoid_control_ws/install/setup.bash
```

### Issue: "ModuleNotFoundError"

**Solution:** Rebuild with symlink install:
```bash
colcon build --symlink-install
```

### Issue: Service not found

**Solution:** Verify service is running:
```bash
ros2 service list | grep status
```

### Issue: Import errors

**Solution:** Check package.xml dependencies:
```xml
<depend>rclpy</depend>
<depend>sensor_msgs</depend>
<depend>std_msgs</depend>
<depend>std_srvs</depend>
```

## Bonus Challenges

**Challenge 1:** Add a second subscriber that logs IMU data
**Challenge 2:** Create a custom service definition for battery status
**Challenge 3:** Implement QoS profiles (Reliable vs Best Effort)
**Challenge 4:** Create a launch file to start all nodes together

## Key Takeaways

- ROS 2 workspaces organize multiple packages
- `colcon build` compiles Python and C++ packages
- Entry points in setup.py create executable commands
- Topics enable asynchronous many-to-many communication
- Services provide synchronous request-response
- ROS 2 tools (topic, service, node) enable debugging

## Next Steps

**Congratulations!** You've built a complete ROS 2 workspace with multiple communication patterns.

**Continue to [Module 2: Digital Twin](../module-02-digital-twin/index.md)** to learn how to:
- Visualize your robot in Gazebo simulation
- Create URDF robot models
- Connect ROS 2 nodes to simulated sensors
- Test control algorithms safely before hardware deployment

---

**Need help?** Review [Lesson 2 (Publishers & Subscribers)](lesson-02-publishers-subscribers.md) for pub-sub patterns or [Lesson 3 (Services & Actions)](lesson-03-services-actions.md) for request-response communication.
