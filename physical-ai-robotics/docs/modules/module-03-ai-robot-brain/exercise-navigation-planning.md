---
id: exercise-navigation-planning
title: Exercise - Navigation Planning for Humanoid Robot
sidebar_position: 5
---

# Exercise: Navigation Planning for Humanoid Robot

## Overview

In this comprehensive exercise, you'll build an autonomous navigation system for your humanoid robot by integrating path planning, reinforcement learning, and behavior trees. You'll configure the Nav2 stack, implement a custom A* global planner, train an RL policy for local obstacle avoidance, and orchestrate navigation with a behavior tree.

By the end, you'll have a humanoid robot that autonomously navigates complex environments in Gazebo, demonstrating the complete AI-robot brain pipeline.

**Estimated time:** 5-6 hours

## Learning Objectives

After completing this exercise, you will be able to:

✅ Configure Nav2 stack for a custom humanoid robot
✅ Implement A* global planner with ROS 2 plugin interface
✅ Train PPO policy for local obstacle avoidance in Gazebo
✅ Create behavior tree for multi-goal navigation tasks
✅ Debug navigation stack using RViz2 and command-line tools
✅ Integrate perception, planning, and control in autonomous system

## Prerequisites

**Required:**
- Module 1 completion (ROS 2 fundamentals)
- Module 2 completion (Gazebo simulation, humanoid URDF)
- Lessons 1-3 of Module 3 (planning, RL, behavior trees)
- Python 3.8+ with NumPy, Matplotlib

**Verify installation:**
```bash
ros2 pkg list | grep nav2      # Should show nav2 packages
python3 -c "import torch"      # PyTorch installed
python3 -c "import py_trees"   # Behavior trees library
```

## Step 1: Install Navigation Stack

### Install Nav2 and Dependencies

```bash
# Nav2 stack (ROS 2 Humble)
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup

# RL training libraries
pip3 install stable-baselines3 gymnasium torch

# Behavior tree libraries
sudo apt install ros-humble-py-trees-ros

# Visualization
sudo apt install ros-humble-rviz2
```

### Create Workspace

```bash
mkdir -p ~/humanoid_nav_ws/src
cd ~/humanoid_nav_ws/src

# Create navigation package
ros2 pkg create --build-type ament_python humanoid_navigation \
  --dependencies rclpy nav2_msgs geometry_msgs sensor_msgs

cd humanoid_navigation
mkdir config launch maps scripts
```

## Step 2: Configure Nav2 for Humanoid

### Create Costmap Configuration

Nav2 uses **costmaps** to represent obstacles for planning.

Create `config/costmap_common.yaml`:

```yaml
footprint: [[0.15, 0.1], [0.15, -0.1], [-0.15, -0.1], [-0.15, 0.1]]  # Humanoid footprint
footprint_padding: 0.05

obstacle_layer:
  plugin: "nav2_costmap_2d::ObstacleLayer"
  enabled: True
  observation_sources: scan
  scan:
    topic: /scan
    max_obstacle_height: 2.0
    clearing: True
    marking: True
    data_type: "LaserScan"

inflation_layer:
  plugin: "nav2_costmap_2d::InflationLayer"
  cost_scaling_factor: 3.0
  inflation_radius: 0.55  # Robot should stay 55cm from obstacles
```

### Create Global Costmap Config

Create `config/global_costmap.yaml`:

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: True
      rolling_window: false  # Static map
      width: 20
      height: 20
      resolution: 0.05  # 5cm cells

      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]

      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True

      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55

      always_send_full_costmap: True
```

### Create Local Costmap Config

Create `config/local_costmap.yaml`:

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: True
      rolling_window: true
      width: 5   # 5m x 5m local window
      height: 5
      resolution: 0.05

      plugins: ["voxel_layer", "inflation_layer"]

      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        publish_voxel_map: True
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"

      always_send_full_costmap: True
```

### Create Nav2 Parameters

Create `config/nav2_params.yaml`:

```yaml
amcl:
  ros__parameters:
    use_sim_time: True
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    base_frame_id: "base_link"
    global_frame_id: "map"
    odom_frame_id: "odom"

bt_navigator:
  ros__parameters:
    use_sim_time: True
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    default_bt_xml_filename: "navigate_w_replanning.xml"

controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.0
    min_theta_velocity_threshold: 0.001

    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      min_vel_x: 0.0
      max_vel_x: 0.5      # Humanoid walking speed
      min_vel_y: 0.0
      max_vel_y: 0.0      # Non-holonomic
      min_speed_theta: 0.0
      max_speed_theta: 1.0
      acc_lim_x: 0.5
      acc_lim_y: 0.0
      acc_lim_theta: 1.0
      decel_lim_x: -0.5
      decel_lim_y: 0.0
      decel_lim_theta: -1.0

planner_server:
  ros__parameters:
    use_sim_time: True
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"  # Will replace with custom A*
      tolerance: 0.5
      use_astar: true
```

## Step 3: Implement Custom A* Planner

### Create A* Plugin

Create `scripts/astar_planner_plugin.py`:

```python
#!/usr/bin/env python3
"""
Custom A* global planner plugin for Nav2
"""
import numpy as np
import heapq
from typing import List, Tuple
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node


class AStarPlanner:
    """A* path planner on occupancy grid."""

    def __init__(self, costmap: OccupancyGrid):
        self.costmap_msg = costmap
        self.width = costmap.info.width
        self.height = costmap.info.height
        self.resolution = costmap.info.resolution
        self.origin_x = costmap.info.origin.position.x
        self.origin_y = costmap.info.origin.position.y

        # Convert to 2D numpy array
        self.grid = np.array(costmap.data).reshape((self.height, self.width))

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """Convert world coordinates to grid indices."""
        grid_x = int((x - self.origin_x) / self.resolution)
        grid_y = int((y - self.origin_y) / self.resolution)
        return (grid_x, grid_y)

    def grid_to_world(self, grid_x: int, grid_y: int) -> Tuple[float, float]:
        """Convert grid indices to world coordinates."""
        x = self.origin_x + (grid_x + 0.5) * self.resolution
        y = self.origin_y + (grid_y + 0.5) * self.resolution
        return (x, y)

    def is_valid(self, grid_x: int, grid_y: int) -> bool:
        """Check if grid cell is valid and free."""
        if grid_x < 0 or grid_x >= self.width or grid_y < 0 or grid_y >= self.height:
            return False
        # Cost < 50 is considered free (0-100 scale, 100=obstacle)
        return self.grid[grid_y, grid_x] < 50

    def heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        """Octile distance heuristic."""
        dx = abs(a[0] - b[0])
        dy = abs(a[1] - b[1])
        return max(dx, dy) + (np.sqrt(2) - 1) * min(dx, dy)

    def get_neighbors(self, pos: Tuple[int, int]) -> List[Tuple[Tuple[int, int], float]]:
        """Get valid 8-connected neighbors with movement costs."""
        x, y = pos
        neighbors = []

        for dx, dy in [(-1,0), (1,0), (0,-1), (0,1),
                       (-1,-1), (-1,1), (1,-1), (1,1)]:
            nx, ny = x + dx, y + dy
            if self.is_valid(nx, ny):
                cost = np.sqrt(2) if dx != 0 and dy != 0 else 1.0
                neighbors.append(((nx, ny), cost))

        return neighbors

    def plan(self, start_world: Tuple[float, float],
             goal_world: Tuple[float, float]) -> List[Tuple[float, float]]:
        """
        Plan path from start to goal using A*.

        Args:
            start_world: (x, y) in world coordinates
            goal_world: (x, y) in world coordinates

        Returns:
            List of (x, y) waypoints in world coordinates
        """
        start = self.world_to_grid(*start_world)
        goal = self.world_to_grid(*goal_world)

        if not self.is_valid(*start) or not self.is_valid(*goal):
            return []  # Invalid start or goal

        open_set = [(0, 0, start)]  # (f_score, counter, position)
        counter = 0
        g_score = {start: 0}
        came_from = {}

        while open_set:
            _, _, current = heapq.heappop(open_set)

            if current == goal:
                # Reconstruct path
                path_grid = [current]
                while current in came_from:
                    current = came_from[current]
                    path_grid.append(current)
                path_grid.reverse()

                # Convert to world coordinates
                path_world = [self.grid_to_world(*p) for p in path_grid]
                return path_world

            for neighbor, cost in self.get_neighbors(current):
                tentative_g = g_score[current] + cost

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + self.heuristic(neighbor, goal)
                    counter += 1
                    heapq.heappush(open_set, (f_score, counter, neighbor))

        return []  # No path found


class AStarPlannerNode(Node):
    """ROS 2 node providing A* planning service."""

    def __init__(self):
        super().__init__('astar_planner')

        # Subscribe to costmap
        self.costmap_sub = self.create_subscription(
            OccupancyGrid,
            '/global_costmap/costmap',
            self.costmap_callback,
            10
        )

        self.costmap = None
        self.planner = None

        self.get_logger().info("A* Planner Node initialized")

    def costmap_callback(self, msg: OccupancyGrid):
        """Update planner with new costmap."""
        self.costmap = msg
        self.planner = AStarPlanner(msg)

    def create_plan(self, start: PoseStamped, goal: PoseStamped) -> Path:
        """Generate path from start to goal."""
        if self.planner is None:
            self.get_logger().warn("No costmap received yet")
            return Path()

        start_xy = (start.pose.position.x, start.pose.position.y)
        goal_xy = (goal.pose.position.x, goal.pose.position.y)

        # Plan path
        waypoints = self.planner.plan(start_xy, goal_xy)

        if not waypoints:
            self.get_logger().error("No path found")
            return Path()

        # Convert to Path message
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()

        for x, y in waypoints:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)

        self.get_logger().info(f"Path created with {len(waypoints)} waypoints")
        return path


def main(args=None):
    rclpy.init(args=args)
    planner = AStarPlannerNode()
    rclpy.spin(planner)
    planner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Step 4: Train RL Policy for Local Planning

### Create Gazebo RL Environment

Create `scripts/rl_local_planner_env.py`:

```python
#!/usr/bin/env python3
"""
Gazebo environment for training local obstacle avoidance with RL
"""
import gymnasium as gym
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class LocalPlannerEnv(gym.Env, Node):
    """RL environment for local obstacle avoidance."""

    def __init__(self):
        gym.Env.__init__(self)
        Node.__init__(self, 'rl_local_planner_env')

        # State: 10 LiDAR ranges + 2 goal direction
        self.observation_space = gym.spaces.Box(
            low=0, high=10.0, shape=(12,), dtype=np.float32
        )

        # Action: linear velocity, angular velocity
        self.action_space = gym.spaces.Box(
            low=np.array([-0.5, -1.0]),
            high=np.array([0.5, 1.0]),
            dtype=np.float32
        )

        # ROS 2 interfaces
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.latest_scan = None
        self.current_pose = None
        self.goal = np.array([5.0, 5.0])  # Target position

    def scan_callback(self, msg: LaserScan):
        """Store latest LiDAR scan."""
        # Downsample to 10 rays
        ranges = np.array(msg.ranges)
        indices = np.linspace(0, len(ranges)-1, 10, dtype=int)
        self.latest_scan = ranges[indices]
        # Replace inf with max range
        self.latest_scan = np.clip(self.latest_scan, 0, 10.0)

    def odom_callback(self, msg: Odometry):
        """Store current robot pose."""
        self.current_pose = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        ])

    def get_observation(self):
        """Construct observation vector."""
        if self.latest_scan is None or self.current_pose is None:
            return np.zeros(12)

        # Goal direction in robot frame
        to_goal = self.goal - self.current_pose
        distance = np.linalg.norm(to_goal)
        angle = np.arctan2(to_goal[1], to_goal[0])

        # Normalize
        distance_norm = min(distance / 10.0, 1.0)
        angle_norm = angle / np.pi

        obs = np.concatenate([
            self.latest_scan / 10.0,  # Normalize LiDAR
            [distance_norm, angle_norm]
        ])
        return obs.astype(np.float32)

    def step(self, action):
        """Execute action, return next state, reward, done."""
        # Send velocity command
        cmd = Twist()
        cmd.linear.x = float(action[0])
        cmd.angular.z = float(action[1])
        self.cmd_pub.publish(cmd)

        # Wait for physics step (0.1s)
        rclpy.spin_once(self, timeout_sec=0.1)

        # Get new observation
        obs = self.get_observation()

        # Compute reward
        reward = self.compute_reward()

        # Check termination
        terminated = self.check_collision() or self.reached_goal()
        truncated = False

        return obs, reward, terminated, truncated, {}

    def compute_reward(self):
        """Reward function for navigation."""
        if self.current_pose is None or self.latest_scan is None:
            return 0.0

        # Distance to goal reward
        dist_to_goal = np.linalg.norm(self.goal - self.current_pose)
        progress_reward = -0.1 * dist_to_goal

        # Collision penalty
        min_distance = np.min(self.latest_scan)
        collision_penalty = -10.0 if min_distance < 0.3 else 0.0

        # Goal reached bonus
        goal_bonus = 100.0 if dist_to_goal < 0.5 else 0.0

        return progress_reward + collision_penalty + goal_bonus

    def check_collision(self):
        """Check if robot collided with obstacle."""
        if self.latest_scan is None:
            return False
        return np.min(self.latest_scan) < 0.25

    def reached_goal(self):
        """Check if robot reached goal."""
        if self.current_pose is None:
            return False
        return np.linalg.norm(self.goal - self.current_pose) < 0.5

    def reset(self, seed=None):
        """Reset environment to initial state."""
        # Reset robot in Gazebo (call service to reset simulation)
        # For simplicity, assume Gazebo is reset externally
        obs = self.get_observation()
        return obs, {}
```

### Train PPO Policy

Create `scripts/train_local_planner.py`:

```python
#!/usr/bin/env python3
"""
Train PPO policy for local obstacle avoidance
"""
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv
from rl_local_planner_env import LocalPlannerEnv
import rclpy


def main():
    rclpy.init()

    # Create vectorized environment
    env = DummyVecEnv([lambda: LocalPlannerEnv()])

    # Create PPO agent
    model = PPO(
        policy="MlpPolicy",
        env=env,
        learning_rate=3e-4,
        n_steps=2048,
        batch_size=64,
        gamma=0.99,
        verbose=1,
        tensorboard_log="./logs/ppo_local_planner/"
    )

    # Train
    print("Training PPO policy for local planning...")
    model.learn(total_timesteps=500_000)

    # Save
    model.save("local_planner_ppo")
    print("Model saved to local_planner_ppo.zip")

    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Run training:**
```bash
cd ~/humanoid_nav_ws/src/humanoid_navigation/scripts
python3 train_local_planner.py
```

**Training time:** ~2-4 hours depending on hardware.

## Step 5: Create Behavior Tree for Navigation

### Multi-Goal Navigation Tree

Create `scripts/navigation_behavior_tree.py`:

```python
#!/usr/bin/env python3
"""
Behavior tree for multi-goal navigation task
"""
import py_trees
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped


class NavigateToGoal(py_trees.behaviour.Behaviour):
    """Navigate to goal using Nav2 action."""

    def __init__(self, name, x, y, theta=0.0):
        super().__init__(name)
        self.goal_x = x
        self.goal_y = y
        self.goal_theta = theta
        self.action_client = None
        self.goal_handle = None

    def setup(self, node):
        self.node = node
        self.action_client = ActionClient(
            node, NavigateToPose, 'navigate_to_pose'
        )

    def initialise(self):
        """Send navigation goal."""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = self.goal_x
        goal_msg.pose.pose.position.y = self.goal_y
        goal_msg.pose.pose.orientation.w = 1.0

        self.action_client.wait_for_server()
        send_goal_future = self.action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self.node, send_goal_future)
        self.goal_handle = send_goal_future.result()

        self.logger.info(f"Navigating to ({self.goal_x}, {self.goal_y})")

    def update(self):
        """Check navigation progress."""
        if self.goal_handle is None:
            return py_trees.common.Status.FAILURE

        if not self.goal_handle.accepted:
            return py_trees.common.Status.FAILURE

        result_future = self.goal_handle.get_result_async()

        if not result_future.done():
            return py_trees.common.Status.RUNNING

        result = result_future.result()
        if result.status == 4:  # SUCCEEDED
            self.logger.info("Navigation succeeded")
            return py_trees.common.Status.SUCCESS
        else:
            self.logger.error("Navigation failed")
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        if self.goal_handle:
            cancel_future = self.goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self.node, cancel_future, timeout_sec=1.0)


class NavigationBehaviorTree(Node):
    """Orchestrate multi-waypoint navigation."""

    def __init__(self):
        super().__init__('navigation_behavior_tree')

        # Define waypoints for patrol route
        waypoints = [
            (2.0, 2.0),
            (5.0, 2.0),
            (5.0, 5.0),
            (2.0, 5.0),
        ]

        # Build behavior tree
        self.root = self.create_patrol_tree(waypoints)
        self.root.setup_with_descendants()

        # Tick at 1 Hz
        self.timer = self.create_timer(1.0, self.tick_tree)

        self.get_logger().info("Navigation behavior tree ready")

    def create_patrol_tree(self, waypoints):
        """Create tree for patrol route."""
        navigate_sequence = py_trees.composites.Sequence(
            name="Patrol Route",
            memory=False,
            children=[
                NavigateToGoal(f"Waypoint {i+1}", x, y)
                for i, (x, y) in enumerate(waypoints)
            ]
        )

        # Loop forever
        root = py_trees.decorators.EternalGuard(
            name="Patrol Forever",
            child=navigate_sequence
        )

        return root

    def tick_tree(self):
        """Tick behavior tree."""
        self.root.tick_once()


def main(args=None):
    rclpy.init(args=args)
    tree = NavigationBehaviorTree()

    try:
        rclpy.spin(tree)
    except KeyboardInterrupt:
        pass

    tree.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Step 6: Launch Complete System

### Create Launch File

Create `launch/navigation.launch.py`:

```python
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('humanoid_navigation')

    # Nav2 bringup
    nav2_launch = IncludeLaunchDescription(
        os.path.join(get_package_share_directory('nav2_bringup'),
                     'launch', 'navigation_launch.py'),
        launch_arguments={
            'params_file': os.path.join(pkg_dir, 'config', 'nav2_params.yaml'),
            'use_sim_time': 'true'
        }.items()
    )

    # Custom A* planner
    astar_planner = Node(
        package='humanoid_navigation',
        executable='astar_planner_plugin.py',
        output='screen'
    )

    # Behavior tree
    behavior_tree = Node(
        package='humanoid_navigation',
        executable='navigation_behavior_tree.py',
        output='screen'
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_dir, 'config', 'nav2.rviz')]
    )

    return LaunchDescription([
        nav2_launch,
        astar_planner,
        behavior_tree,
        rviz,
    ])
```

### Update package.xml

Add dependencies:

```xml
<depend>nav2_msgs</depend>
<depend>geometry_msgs</depend>
<depend>sensor_msgs</depend>
<depend>nav_msgs</depend>
<depend>ros_gz_bridge</depend>
<depend>py_trees_ros</depend>
```

### Update setup.py

```python
from setuptools import setup
from glob import glob
import os

package_name = 'humanoid_navigation'

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
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'astar_planner_plugin = scripts.astar_planner_plugin:main',
            'navigation_behavior_tree = scripts.navigation_behavior_tree:main',
        ],
    },
)
```

## Step 7: Build and Test

### Build Workspace

```bash
cd ~/humanoid_nav_ws
colcon build --symlink-install
source install/setup.bash
```

### Launch Gazebo with Humanoid

**Terminal 1 - Gazebo:**
```bash
source ~/humanoid_sim_ws/install/setup.bash
ros2 launch humanoid_description spawn_humanoid.launch.py
```

### Launch Navigation Stack

**Terminal 2 - Navigation:**
```bash
source ~/humanoid_nav_ws/install/setup.bash
ros2 launch humanoid_navigation navigation.launch.py
```

### Send Navigation Goal

**Terminal 3 - Test navigation:**
```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 5.0, y: 5.0}, orientation: {w: 1.0}}}"
```

**Expected:**
- RViz2 shows global path (green line)
- Robot follows path in Gazebo
- Local costmap updates around robot
- Robot reaches goal and stops

## Acceptance Criteria

Your autonomous navigation system is complete when:

- [ ] Nav2 stack launches without errors
- [ ] RViz2 displays global costmap, local costmap, and robot model
- [ ] A* planner generates paths avoiding obstacles
- [ ] Robot successfully navigates to published goal poses
- [ ] Behavior tree cycles through patrol waypoints
- [ ] RL local planner avoids dynamic obstacles (if trained)
- [ ] `ros2 topic list` shows `/global_costmap/costmap`, `/local_costmap/costmap`, `/plan`
- [ ] Robot stops when goal reached (position error < 0.5m)

## Troubleshooting

### Nav2 won't start

**Solution:** Check parameter files match ROS 2 Humble format:
```bash
ros2 param describe nav2_controller controller_frequency
```

### Planner finds no path

**Solution:** Check costmap in RViz2:
- Obstacles shown correctly?
- Start/goal in free space?
- Inflation radius too large?

### Robot doesn't move

**Solution:** Verify cmd_vel topic:
```bash
ros2 topic echo /cmd_vel
# Should show velocity commands
```

### RViz2 shows "No transform from map to base_link"

**Solution:** Start AMCL or publish static transform:
```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom
```

## Bonus Challenges

**Challenge 1:** Add semantic waypoints (charging station, table, door) to behavior tree
**Challenge 2:** Implement recovery behaviors (backup, spin) when stuck
**Challenge 3:** Train RL policy to handle stairs and uneven terrain
**Challenge 4:** Add vision-based obstacle detection (replace LiDAR)

## Key Takeaways

- **Nav2 stack provides production-ready navigation for ROS 2**
- **Costmaps represent obstacles for planning (global + local)**
- **A* global planner finds optimal paths on grid**
- **Local planner (DWA or RL) executes path with obstacle avoidance**
- **Behavior trees orchestrate multi-goal navigation tasks**
- **RViz2 visualizes costmaps, paths, and robot state for debugging**
- **RL policies can replace hand-tuned local planners for complex environments**

## Next Steps

**Congratulations!** You've built a complete autonomous navigation system integrating planning, learning, and control.

**Continue to [Module 4: Vision-Language-Action](../module-04-vision-language-action/index.md)** to learn:
- Vision transformers for robot perception
- Large language models for task planning
- Vision-language-action (VLA) models
- Multimodal robot control with natural language

---

**Need help?** Review [Lesson 1 (Planning)](lesson-01-planning-algorithms.md), [Lesson 2 (RL)](lesson-02-reinforcement-learning.md), or [Lesson 3 (Behavior Trees)](lesson-03-behavior-trees.md).
