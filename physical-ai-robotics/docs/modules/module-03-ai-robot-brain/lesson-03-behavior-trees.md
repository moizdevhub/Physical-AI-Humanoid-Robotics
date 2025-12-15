---
id: lesson-03-behavior-trees
title: Lesson 3 - Behavior Trees
sidebar_position: 4
---

# Lesson 3: Behavior Trees

## Introduction

**Behavior Trees (BTs)** provide a modular, hierarchical framework for orchestrating robot tasks. Unlike finite state machines, behavior trees are reactive, composable, and easier to scale to complex behaviors.

For humanoid robots performing multi-step tasks (navigate to table, pick up object, deliver to person), behavior trees enable clear task decomposition while maintaining responsiveness to environmental changes.

In this lesson, you'll learn behavior tree fundamentals, implement nodes in Python with py_trees, and integrate with ROS 2 action servers for robot control.

## The Task Orchestration Problem

### Why Not Simple Scripting?

**Naive approach:**
```python
def deliver_object():
    navigate_to_table()
    pick_up_object()
    navigate_to_person()
    hand_over_object()
```

**Problems:**
- **No error handling:** What if navigation fails?
- **No reactivity:** Can't interrupt if emergency stop pressed
- **No modularity:** Hard to reuse navigate_to_table() in other tasks
- **No monitoring:** Can't query task progress

### Finite State Machines (FSMs)

**State-based approach:**
```
States: IDLE → NAVIGATING → PICKING → DELIVERING → IDLE
Transitions: success/failure conditions
```

**Pros:**
- Clear states
- Explicit transitions

**Cons:**
- **Transition explosion:** N states → O(N²) possible transitions
- **Not composable:** Hard to nest FSMs
- **Not reactive:** Must manually add "check emergency stop" in every state

### Behavior Trees Solution

**Hierarchical decomposition:**
```
Root
└── Sequence: "Deliver Object"
    ├── Navigate to Table
    ├── Pick Up Object
    ├── Navigate to Person
    └── Hand Over Object
```

**Advantages:**
- **Reactive:** Tree ticked every timestep, responds immediately
- **Composable:** Subtrees can be reused
- **Readable:** Visual structure matches task logic
- **Scalable:** Add behaviors without touching existing code

## Behavior Tree Basics

### Tree Structure

**Nodes:**
- **Leaf nodes:** Actions or conditions (actual robot commands)
- **Internal nodes:** Control flow (sequence, selector, parallel)

**Execution:**
- Tree is **ticked** at regular intervals (e.g., 10 Hz)
- Each tick propagates from root to leaves
- Nodes return status: `SUCCESS`, `FAILURE`, or `RUNNING`

**Example:**
```
Root (tick) → Sequence (tick) → Action1 (tick) → returns RUNNING
Sequence pauses, next tick continues from Action1
```

### Node Status

**SUCCESS:**
- Action completed successfully
- Condition evaluated to True
- Example: "Navigate to goal" reached destination

**FAILURE:**
- Action failed
- Condition evaluated to False
- Example: "Pick object" but object not found

**RUNNING:**
- Action in progress
- Waiting for result
- Example: "Navigate to goal" still moving

## Core Node Types

### 1. Action Node (Leaf)

**Executes a robot command or ROS 2 action.**

**Behavior:**
- First tick: Start action (send goal to action server)
- Subsequent ticks: Check status
- Returns: `RUNNING` while executing, `SUCCESS`/`FAILURE` when done

**Example:**
```python
import py_trees
import rclpy
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

class NavigateToGoal(py_trees.behaviour.Behaviour):
    """Navigate to a goal position using Nav2."""

    def __init__(self, name, goal_x, goal_y):
        super().__init__(name)
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.action_client = None
        self.goal_handle = None

    def setup(self, node):
        """Initialize action client (called once)."""
        self.node = node
        self.action_client = ActionClient(
            node, NavigateToPose, 'navigate_to_pose'
        )
        self.logger.debug(f"Setup {self.name}")

    def initialise(self):
        """Send navigation goal (called when node first ticked)."""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.pose.position.x = self.goal_x
        goal_msg.pose.pose.position.y = self.goal_y
        goal_msg.pose.pose.orientation.w = 1.0

        self.action_client.wait_for_server()
        self.goal_handle = self.action_client.send_goal_async(goal_msg)
        self.logger.info(f"Sent navigation goal: ({self.goal_x}, {self.goal_y})")

    def update(self):
        """Check action status (called every tick)."""
        if self.goal_handle is None:
            return py_trees.common.Status.FAILURE

        if not self.goal_handle.done():
            return py_trees.common.Status.RUNNING

        result = self.goal_handle.result()
        if result.status == 4:  # SUCCEEDED
            self.logger.info("Navigation succeeded")
            return py_trees.common.Status.SUCCESS
        else:
            self.logger.error("Navigation failed")
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        """Cleanup when node stops (success, failure, or interrupted)."""
        if self.goal_handle and not self.goal_handle.done():
            self.goal_handle.cancel()
        self.logger.debug(f"Terminated with status {new_status}")
```

### 2. Condition Node (Leaf)

**Checks a condition (no side effects).**

**Behavior:**
- Evaluates condition immediately
- Returns: `SUCCESS` if true, `FAILURE` if false

**Example:**
```python
class IsBatteryLow(py_trees.behaviour.Behaviour):
    """Check if robot battery is below threshold."""

    def __init__(self, name, threshold=20.0):
        super().__init__(name)
        self.threshold = threshold

    def setup(self, node):
        self.node = node
        self.battery_level = 100.0  # Subscribe to battery topic in real implementation

    def update(self):
        # Check battery level (would read from ROS 2 topic in real code)
        if self.battery_level < self.threshold:
            self.logger.warning(f"Battery low: {self.battery_level}%")
            return py_trees.common.Status.SUCCESS  # Condition is TRUE
        else:
            return py_trees.common.Status.FAILURE  # Condition is FALSE
```

### 3. Sequence Node (Composite)

**Executes children in order, succeeds only if ALL children succeed.**

**Behavior:**
- Ticks children left-to-right
- If child returns `SUCCESS`: move to next child
- If child returns `FAILURE`: return `FAILURE` (stop sequence)
- If child returns `RUNNING`: return `RUNNING` (pause sequence)
- Returns `SUCCESS` if all children succeed

**Analogy:** Logical AND

**Example:**
```
Sequence: "Make Coffee"
├── Fill Water Reservoir    → SUCCESS
├── Add Coffee Grounds      → SUCCESS
└── Press Start Button      → SUCCESS
→ Sequence returns SUCCESS
```

**Code:**
```python
root = py_trees.composites.Sequence(
    name="Make Coffee",
    memory=False,  # Restart from first child each tick
    children=[
        FillWaterReservoir("Fill Water"),
        AddCoffeeGrounds("Add Grounds"),
        PressStartButton("Start"),
    ]
)
```

### 4. Selector Node (Composite)

**Tries children in order, succeeds if ANY child succeeds.**

**Behavior:**
- Ticks children left-to-right
- If child returns `SUCCESS`: return `SUCCESS` (stop trying)
- If child returns `FAILURE`: move to next child
- If child returns `RUNNING`: return `RUNNING` (pause)
- Returns `FAILURE` if all children fail

**Analogy:** Logical OR, fallback behavior

**Example:**
```
Selector: "Enter Building"
├── Use Door           → FAILURE (door locked)
├── Use Side Entrance  → FAILURE (also locked)
└── Call for Help      → SUCCESS
→ Selector returns SUCCESS
```

**Code:**
```python
root = py_trees.composites.Selector(
    name="Enter Building",
    memory=False,
    children=[
        UseDoor("Main Door"),
        UseSideEntrance("Side Door"),
        CallForHelp("Call"),
    ]
)
```

### 5. Parallel Node (Composite)

**Executes all children simultaneously.**

**Policies:**
- **SuccessOnAll:** Succeeds when ALL children succeed
- **SuccessOnOne:** Succeeds when ANY child succeeds
- **SuccessOnSelected:** Succeeds when specific children succeed

**Use case:** Concurrent tasks (walk WHILE monitoring battery)

**Example:**
```
Parallel(SuccessOnAll): "Walk and Monitor"
├── Navigate to Goal        → RUNNING
└── Monitor Battery Level   → RUNNING
→ Both must succeed for parallel to succeed
```

**Code:**
```python
root = py_trees.composites.Parallel(
    name="Walk and Monitor",
    policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
    children=[
        NavigateToGoal("Navigate", x=5, y=5),
        MonitorBattery("Monitor"),
    ]
)
```

### 6. Decorator Nodes

**Modify child behavior (wraps single child).**

**Common decorators:**

**Inverter:**
- Flips child status: `SUCCESS` ↔ `FAILURE`
- Use case: "Not at goal" condition

**Retry:**
- Retries child N times on failure
- Use case: Unreliable actions (grasp object)

**Timeout:**
- Returns `FAILURE` if child runs too long
- Use case: Prevent infinite loops

**Example:**
```python
# Invert condition
is_not_at_goal = py_trees.decorators.Inverter(
    name="Not At Goal",
    child=IsAtGoal("Check Goal")
)

# Retry action 3 times
retry_grasp = py_trees.decorators.Retry(
    name="Retry Grasp",
    child=GraspObject("Grasp"),
    num_failures=3
)

# Timeout after 30 seconds
timeout_nav = py_trees.decorators.Timeout(
    name="Nav Timeout",
    child=NavigateToGoal("Navigate", 5, 5),
    duration=30.0
)
```

## Complete Example: Delivery Task

### Task Description

**Goal:** Humanoid robot delivers object from table to person.

**Steps:**
1. Navigate to table
2. Pick up object
3. Navigate to person
4. Hand over object
5. Return to charging station if battery low

### Behavior Tree Structure

```
Root: Selector
├── Sequence: "Handle Low Battery"
│   ├── Condition: Battery < 20%
│   └── Action: Navigate to Charger
└── Sequence: "Deliver Object"
    ├── Action: Navigate to Table
    ├── Action: Pick Up Object
    ├── Action: Navigate to Person
    ├── Action: Hand Over Object
    └── Action: Navigate Home
```

**Logic:**
- If battery low: Go to charger (takes priority)
- Otherwise: Execute delivery sequence

### Implementation

```python
#!/usr/bin/env python3
"""
Behavior tree for humanoid delivery task
"""
import py_trees
import rclpy
from rclpy.node import Node

class DeliveryBehaviorTree(Node):
    """Humanoid delivery task orchestrator."""

    def __init__(self):
        super().__init__('delivery_behavior_tree')

        # Create behavior tree
        self.root = self.create_tree()

        # Setup tree
        self.root.setup_with_descendants()

        # Tick tree at 10 Hz
        self.timer = self.create_timer(0.1, self.tick_tree)

        self.get_logger().info("Behavior tree initialized")

    def create_tree(self):
        """Build behavior tree structure."""

        # Low battery handler (priority)
        low_battery_handler = py_trees.composites.Sequence(
            name="Handle Low Battery",
            memory=False,
            children=[
                IsBatteryLow("Battery Check", threshold=20.0),
                NavigateToGoal("Go to Charger", x=0, y=0),
            ]
        )

        # Main delivery sequence
        delivery_sequence = py_trees.composites.Sequence(
            name="Deliver Object",
            memory=False,
            children=[
                NavigateToGoal("Go to Table", x=5, y=2),
                PickUpObject("Pick Object"),
                NavigateToGoal("Go to Person", x=8, y=8),
                HandOverObject("Hand Over"),
                NavigateToGoal("Go Home", x=0, y=0),
            ]
        )

        # Root selector: Try low battery handler first, then delivery
        root = py_trees.composites.Selector(
            name="Root",
            memory=False,
            children=[
                low_battery_handler,
                delivery_sequence,
            ]
        )

        return root

    def tick_tree(self):
        """Tick tree once (called at 10 Hz)."""
        self.root.tick_once()

        # Log status
        status = self.root.status
        if status == py_trees.common.Status.SUCCESS:
            self.get_logger().info("Task completed successfully")
            # Could reset tree or shutdown
        elif status == py_trees.common.Status.FAILURE:
            self.get_logger().error("Task failed")


def main(args=None):
    rclpy.init(args=args)
    tree_node = DeliveryBehaviorTree()

    try:
        rclpy.spin(tree_node)
    except KeyboardInterrupt:
        pass
    finally:
        tree_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Behavior Trees vs Finite State Machines

| Feature | Behavior Trees | Finite State Machines |
|---------|----------------|------------------------|
| **Structure** | Hierarchical tree | Flat states with transitions |
| **Modularity** | High (composable subtrees) | Low (tangled transitions) |
| **Scalability** | Excellent (add nodes) | Poor (transition explosion) |
| **Reactivity** | Built-in (ticked every frame) | Manual (add checks in states) |
| **Readability** | Visual, self-documenting | Spaghetti for complex tasks |
| **State History** | Optional (memory flag) | Implicit (current state) |

### When to Use FSMs

**Good for:**
- Simple tasks with few states (< 5 states)
- Clear, discrete modes (IDLE, ACTIVE, ERROR)
- State history important (know exactly which state robot was in)

**Example:** Motor controller (OFF → IDLE → ACTIVE → ERROR)

### When to Use Behavior Trees

**Good for:**
- Complex tasks with many steps
- Hierarchical decomposition needed
- Reactive behaviors (respond to sensor inputs)
- Reusable behaviors across tasks

**Example:** Humanoid serving drinks at a party (many subtasks, dynamic environment)

## Advanced Techniques

### Blackboard (Shared Memory)

**Problem:** Nodes need to share data (e.g., detected object pose).

**Solution:** Blackboard is shared key-value store.

```python
# Write to blackboard
class DetectObject(py_trees.behaviour.Behaviour):
    def update(self):
        object_pose = self.detect()  # Computer vision
        self.blackboard.set("object_pose", object_pose)
        return py_trees.common.Status.SUCCESS

# Read from blackboard
class GraspObject(py_trees.behaviour.Behaviour):
    def update(self):
        object_pose = self.blackboard.get("object_pose")
        self.send_grasp_command(object_pose)
        return py_trees.common.Status.SUCCESS
```

### Subtree Reuse

**Modular subtrees can be instantiated multiple times.**

```python
def create_navigation_subtree(name, x, y):
    """Reusable navigation with fallback."""
    return py_trees.composites.Selector(
        name=name,
        children=[
            NavigateToGoal(f"{name} Primary", x, y),
            NavigateToGoal(f"{name} Fallback", x+1, y),  # Try nearby point
        ]
    )

# Use in multiple places
tree = py_trees.composites.Sequence(
    name="Multi-Stop Delivery",
    children=[
        create_navigation_subtree("Go to Stop 1", 5, 2),
        create_navigation_subtree("Go to Stop 2", 8, 8),
        create_navigation_subtree("Go Home", 0, 0),
    ]
)
```

### Tree Visualization

**py_trees provides rendering:**

```python
# Print ASCII tree
py_trees.display.print_ascii_tree(root, show_status=True)

# Output:
# [->] Root
#     [-] Sequence: Deliver Object
#         [✓] Navigate to Table
#         [↻] Pick Up Object
#         [ ] Navigate to Person
#         [ ] Hand Over

# Generate DOT graph (visualize in Graphviz)
py_trees.display.render_dot_tree(root, name="delivery_tree")
```

## ROS 2 Integration

### Using py_trees_ros

**py_trees_ros bridge connects behavior trees to ROS 2.**

**Installation:**
```bash
sudo apt install ros-humble-py-trees-ros
```

**Key features:**
- Action clients as behavior tree actions
- Topic subscribers as conditions
- Service calls as actions
- Blackboard synchronized with ROS 2 topics

**Example:**
```python
from py_trees_ros import action_clients

# Wrap ROS 2 action as behavior tree node
navigate_action = action_clients.FromConstant(
    action_type=NavigateToPose,
    action_name="navigate_to_pose",
    action_goal=goal_msg,
    name="Navigate"
)
```

## Debugging Behavior Trees

### Common Issues

**Node never returns:**
- Check `update()` method returns status
- Ensure action servers respond

**Sequence fails unexpectedly:**
- Log each child's status
- Check conditions evaluate correctly

**Tree doesn't react to changes:**
- Verify tick rate (should be 10+ Hz)
- Check blackboard updates

### Debugging Tools

**1. Status logging:**
```python
self.logger.info(f"Node {self.name}: {self.status}")
```

**2. Tree snapshots:**
```python
snapshot = py_trees.visitors.SnapshotVisitor()
root.visit(snapshot)
print(snapshot.visited)
```

**3. RViz visualization:**
- py_trees_ros publishes tree state to `/tree/status` topic
- Custom RViz plugin can display live tree

## Key Takeaways

- **Behavior trees hierarchically decompose complex tasks**
- **Nodes return SUCCESS, FAILURE, or RUNNING each tick**
- **Sequence: Execute children in order (AND logic)**
- **Selector: Try children until one succeeds (OR logic)**
- **Parallel: Execute children concurrently**
- **Decorators: Modify child behavior (invert, retry, timeout)**
- **Blackboard: Shared memory for inter-node communication**
- **More modular and reactive than finite state machines**
- **py_trees + py_trees_ros enable ROS 2 integration**

## Next Steps

In the **[Exercise: Navigation Planning for Humanoid Robot](exercise-navigation-planning.md)**, you will:
- Configure Nav2 stack for your humanoid robot
- Implement a custom A* global planner
- Train an RL policy for local obstacle avoidance
- Create a behavior tree coordinating navigation and manipulation
- Test end-to-end autonomous navigation in Gazebo

**Before starting the exercise**, make sure you can:
- Explain the difference between Sequence and Selector nodes
- Implement a simple behavior tree with py_trees
- Understand how behavior trees integrate with ROS 2 actions
- Recognize when behavior trees are better than FSMs

---

**Estimated time for Exercise:** 5-6 hours (Nav2 setup + RL training + behavior tree)
