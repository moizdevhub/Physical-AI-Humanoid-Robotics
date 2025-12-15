---
id: lesson-03-services-actions
title: Lesson 3 - Services & Actions
sidebar_position: 4
---

# Lesson 3: Services & Actions

## Introduction

While topics are great for streaming data, robotics systems also need **request-response** communication and **long-running tasks**. ROS 2 provides two patterns for these scenarios:

- **Services**: Synchronous request-response (like function calls)
- **Actions**: Asynchronous long-running tasks with feedback and cancellation

In this lesson, you learn when to use each pattern and how to implement them for humanoid robot control.

## Services: Request-Response Pattern

### What Are Services?

A **service** is like a remote function call:
1. **Client** sends a request
2. **Server** processes the request
3. **Server** sends back a response
4. **Client** receives the response (blocking)

**When to use services:**
- Balance calibration ("Calibrate IMU now" → "Calibration complete")
- Status queries ("What is battery level?" → "75%")
- Configuration changes ("Set max speed to 1.0 m/s" → "Confirmed")
- One-time commands ("Take photo" → "Photo saved")

**When NOT to use services:**
- Continuous data streams (use Topics)
- Long tasks that need progress feedback (use Actions)

### Service Example: Balance Calibration

A humanoid robot needs to calibrate its IMU sensor before walking. This is a one-time operation with a clear result.

**Service definition (simplified):**
```
# Request
bool start_calibration

---
# Response
bool success
string message
float32[] calibration_offsets
```

**Client code structure:**
```python
from rclpy.node import Node

class BalanceCalibrationClient(Node):
    def __init__(self):
        super().__init__('calibration_client')
        self.client = self.create_client(
            CalibrateBalance,
            'calibrate_balance'
        )

    def send_request(self):
        request = CalibrateBalance.Request()
        request.start_calibration = True

        future = self.client.call_async(request)
        # Wait for response...
```

**Server code structure:**
```python
class BalanceCalibrationServer(Node):
    def __init__(self):
        super().__init__('calibration_server')
        self.service = self.create_service(
            CalibrateBalance,
            'calibrate_balance',
            self.calibrate_callback
        )

    def calibrate_callback(self, request, response):
        # Perform calibration...
        response.success = True
        response.message = "Calibration complete"
        return response
```

## Actions: Long-Running Tasks

### What Are Actions?

An **action** is for tasks that take time and need progress updates:
1. **Client** sends a goal
2. **Server** starts execution
3. **Server** sends periodic feedback
4. **Client** can cancel anytime
5. **Server** sends final result

**When to use actions:**
- Navigation ("Go to coordinates X,Y" with progress updates)
- Arm movement ("Move gripper to pose" with trajectory feedback)
- Multi-step tasks ("Stand up from sitting" with phase updates)

**When NOT to use actions:**
- Instant operations (use Services)
- Continuous data (use Topics)

### Action Example: Navigation to Goal

A humanoid robot walks to a target position. This takes several seconds and the client wants progress updates.

**Action definition (simplified):**
```
# Goal
float32 target_x
float32 target_y

---
# Result
bool success
float32 final_x
float32 final_y

---
# Feedback
float32 current_x
float32 current_y
float32 distance_remaining
```

**Client code structure:**
```python
from rclpy.action import ActionClient

class NavigationClient(Node):
    def __init__(self):
        super().__init__('nav_client')
        self.action_client = ActionClient(
            self,
            NavigateToGoal,
            'navigate_to_goal'
        )

    def send_goal(self, x, y):
        goal = NavigateToGoal.Goal()
        goal.target_x = x
        goal.target_y = y

        self.action_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback
        )

    def feedback_callback(self, feedback):
        print(f'Distance remaining: {feedback.distance_remaining}m')
```

**Server code structure:**
```python
from rclpy.action import ActionServer

class NavigationServer(Node):
    def __init__(self):
        super().__init__('nav_server')
        self.action_server = ActionServer(
            self,
            NavigateToGoal,
            'navigate_to_goal',
            self.execute_callback
        )

    def execute_callback(self, goal_handle):
        # Move robot toward goal
        while distance_remaining > 0.1:
            # Publish feedback
            feedback = NavigateToGoal.Feedback()
            feedback.distance_remaining = distance_remaining
            goal_handle.publish_feedback(feedback)

        goal_handle.succeed()
        return result
```

## Comparison: Topics vs Services vs Actions

| Feature | Topics | Services | Actions |
|---------|--------|----------|---------|
| **Pattern** | Pub-Sub | Request-Response | Goal-Feedback-Result |
| **Timing** | Asynchronous | Synchronous | Asynchronous |
| **Direction** | One-way | Two-way | Two-way + feedback |
| **Cancellation** | N/A | No | Yes |
| **Feedback** | No | No | Yes |
| **Use case** | Sensor streams | Status queries | Long tasks |

### Decision Tree

**Need continuous data stream?** → Use Topics

**Need instant response?** → Use Services

**Need progress updates or cancellation?** → Use Actions

## Humanoid Robot Communication Patterns

**Typical topic usage:**
- `/imu/data` - IMU sensor stream (100 Hz)
- `/joint_states` - Joint positions (50 Hz)
- `/camera/image` - Camera feed (30 Hz)

**Typical service usage:**
- `/calibrate_imu` - One-time calibration
- `/get_battery_status` - Query current charge
- `/set_gait_params` - Configure walking speed

**Typical action usage:**
- `/navigate_to_goal` - Walk to position
- `/stand_up` - Stand from sitting position
- `/grasp_object` - Pick up an object

## Key Takeaways

- **Services** provide synchronous request-response
- **Actions** handle long-running tasks with feedback
- **Topics** remain best for continuous data streams
- Choose the right pattern based on timing and feedback needs
- Humanoid robots use all three patterns simultaneously

## Next Steps

**Congratulations!** You have completed Module 1: Robotic Nervous System.

You now understand:
- ROS 2 architecture and core concepts
- Publisher-subscriber pattern for data streaming
- Services for request-response communication
- Actions for long-running tasks

**Next module:** [Module 2: Digital Twin](../module-02-digital-twin/index.md) - Learn to simulate humanoid robots in Gazebo

---

**Estimated time for Module 2:** 10-12 hours over 3 weeks
