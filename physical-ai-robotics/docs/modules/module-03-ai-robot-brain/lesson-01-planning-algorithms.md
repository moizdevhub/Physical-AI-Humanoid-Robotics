---
id: lesson-01-planning-algorithms
title: Lesson 1 - Planning Algorithms
sidebar_position: 2
---

# Lesson 1: Planning Algorithms

## Introduction

**Motion planning** is the process of finding a collision-free path from a start position to a goal position. For humanoid robots navigating indoor environments, path planning algorithms must handle obstacles, tight spaces, and dynamic changes while respecting robot constraints.

In this lesson, you'll learn classical planning algorithms used in robotics, understand when to use grid-based vs. sampling-based methods, and implement A* pathfinding for a humanoid robot.

## The Planning Problem

### Formal Definition

**Given:**
- Start configuration: `q_start` (x, y, θ)
- Goal configuration: `q_goal` (x, y, θ)
- Obstacles: `O = {O_1, O_2, ..., O_n}`
- Robot model: dimensions, kinematics

**Find:**
- Path: `P = [q_start, q_1, q_2, ..., q_goal]`

**Constraints:**
- Collision-free: No overlap with obstacles
- Kinematically feasible: Respects turning radius, joint limits
- Optimal: Minimizes cost (distance, time, energy)

### Humanoid Robot Example

For a humanoid navigating an office:

```
Start: Robot at desk (x=0, y=0, θ=0°)
Goal: Robot at door (x=10, y=8, θ=90°)
Obstacles: Desks, chairs, walls
Constraints: Cannot rotate in place (non-holonomic)
```

## Configuration Space (C-Space)

### What is C-Space?

**Configuration space** transforms the planning problem:
- **Workspace**: Physical 3D environment (obstacles as shapes)
- **C-space**: Abstract space where robot is a point

**Why C-space?**
- Simplifies collision checking (point vs. shape)
- Standard algorithms work on point robots

### C-Space Construction

**2D Robot (x, y position):**
```
Workspace obstacle: 1m × 1m box at (5, 5)
Robot: 0.3m radius circle
C-space obstacle: 1.6m × 1.6m box at (5, 5)  # Grown by robot radius
```

**3D Humanoid (x, y, θ):**
- Position: (x, y)
- Orientation: θ
- C-space is 3D: collisions depend on orientation

## Grid-Based Planning

Grid-based methods discretize C-space into cells.

### Dijkstra's Algorithm

Finds shortest path by exploring all directions equally.

**Algorithm:**
1. Initialize: Distance to start = 0, all others = ∞
2. Visit cell with smallest distance
3. Update neighbors: `dist[neighbor] = min(dist[neighbor], dist[current] + cost)`
4. Repeat until goal reached

**Pros:**
- Guaranteed optimal path
- Works on any weighted graph

**Cons:**
- Explores all directions (slow)
- No goal bias

### A* (A-Star) Algorithm

**Key insight:** Use heuristic to guide search toward goal.

**Cost function:**
```
f(n) = g(n) + h(n)
```
- `g(n)`: Cost from start to n (known)
- `h(n)`: Estimated cost from n to goal (heuristic)
- `f(n)`: Total estimated cost

**Heuristics for 2D grid:**

**Euclidean distance:**
```python
h(n) = sqrt((n.x - goal.x)^2 + (n.y - goal.y)^2)
```
- Admissible: Never overestimates
- Good for any-angle movement

**Manhattan distance:**
```python
h(n) = abs(n.x - goal.x) + abs(n.y - goal.y)
```
- Admissible for grid movement (4-connected)
- Faster to compute

**Octile distance:**
```python
dx = abs(n.x - goal.x)
dy = abs(n.y - goal.y)
h(n) = max(dx, dy) + (sqrt(2) - 1) * min(dx, dy)
```
- Best for 8-connected grids (diagonal movement)

### A* Implementation

```python
#!/usr/bin/env python3
"""
A* path planning for grid-based navigation
"""
import heapq
import numpy as np
from typing import List, Tuple

class AStarPlanner:
    """A* pathfinding on 2D occupancy grid."""

    def __init__(self, grid: np.ndarray):
        """
        Args:
            grid: 2D array where 0=free, 1=obstacle
        """
        self.grid = grid
        self.height, self.width = grid.shape

    def heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        """Octile distance heuristic."""
        dx = abs(a[0] - b[0])
        dy = abs(a[1] - b[1])
        return max(dx, dy) + (np.sqrt(2) - 1) * min(dx, dy)

    def get_neighbors(self, pos: Tuple[int, int]) -> List[Tuple[int, int, float]]:
        """Return valid neighbors with movement costs."""
        x, y = pos
        neighbors = []

        # 8-connected grid
        for dx, dy in [(-1,0), (1,0), (0,-1), (0,1),  # Cardinal
                       (-1,-1), (-1,1), (1,-1), (1,1)]:  # Diagonal
            nx, ny = x + dx, y + dy

            # Check bounds
            if 0 <= nx < self.width and 0 <= ny < self.height:
                # Check collision
                if self.grid[ny, nx] == 0:
                    cost = np.sqrt(2) if dx != 0 and dy != 0 else 1.0
                    neighbors.append(((nx, ny), cost))

        return neighbors

    def plan(self, start: Tuple[int, int], goal: Tuple[int, int]) -> List[Tuple[int, int]]:
        """
        Find shortest path from start to goal using A*.

        Returns:
            List of (x, y) positions from start to goal, or empty list if no path.
        """
        # Priority queue: (f_score, counter, position)
        open_set = [(0, 0, start)]
        counter = 0

        # Track best g_score to each position
        g_score = {start: 0}

        # Track path
        came_from = {}

        while open_set:
            _, _, current = heapq.heappop(open_set)

            # Goal reached
            if current == goal:
                return self._reconstruct_path(came_from, current)

            # Explore neighbors
            for neighbor, cost in self.get_neighbors(current):
                tentative_g = g_score[current] + cost

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    # Better path found
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + self.heuristic(neighbor, goal)

                    counter += 1
                    heapq.heappush(open_set, (f_score, counter, neighbor))

        # No path found
        return []

    def _reconstruct_path(self, came_from: dict, current: Tuple[int, int]) -> List[Tuple[int, int]]:
        """Backtrack from goal to start."""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path


# Example usage
if __name__ == '__main__':
    # Create 20x20 grid with obstacles
    grid = np.zeros((20, 20))
    grid[5:15, 10] = 1  # Vertical wall
    grid[10, 5:15] = 1  # Horizontal wall

    planner = AStarPlanner(grid)
    path = planner.plan(start=(0, 0), goal=(19, 19))

    if path:
        print(f"Path found with {len(path)} waypoints")
        print(f"Path length: {sum(np.linalg.norm(np.array(path[i+1]) - np.array(path[i])) for i in range(len(path)-1)):.2f}")
    else:
        print("No path found")
```

### Grid Resolution Trade-off

**Fine grid (0.05m cells):**
- Pro: Accurate paths, tight spaces
- Con: Large memory, slow search

**Coarse grid (0.5m cells):**
- Pro: Fast search, low memory
- Con: Misses narrow passages, jagged paths

**Typical:** 0.1-0.2m for indoor navigation

## Sampling-Based Planning

Sampling-based methods avoid discretization by randomly sampling C-space.

### Rapidly-Exploring Random Tree (RRT)

**Idea:** Grow a tree from start toward goal by random sampling.

**Algorithm:**
1. Initialize tree with start configuration
2. Sample random configuration `q_rand`
3. Find nearest tree node `q_near`
4. Steer toward `q_rand` by step size `ε`: `q_new = q_near + ε * (q_rand - q_near)`
5. If collision-free, add `q_new` to tree
6. Repeat until tree reaches goal

**Visualization:**
```
Tree growth:
Start ●
      ├─● (expand toward random sample)
      │ └─●
      └─● (expand toward random sample)
        └─● Goal reached!
```

**Properties:**
- **Probabilistically complete:** Will find path if one exists (given infinite time)
- **Not optimal:** First path found is returned
- **Good for high-dimensional spaces:** Works in 10+ dimensions

### RRT Implementation

```python
#!/usr/bin/env python3
"""
RRT path planning for continuous configuration spaces
"""
import numpy as np
from typing import List, Tuple, Optional

class RRTNode:
    """Node in RRT search tree."""
    def __init__(self, config: np.ndarray):
        self.config = config  # Configuration (x, y) or (x, y, θ)
        self.parent: Optional[RRTNode] = None

class RRTPlanner:
    """Rapidly-Exploring Random Tree planner."""

    def __init__(self, bounds: List[Tuple[float, float]], step_size: float = 0.5):
        """
        Args:
            bounds: [(x_min, x_max), (y_min, y_max), ...]
            step_size: Maximum distance to extend tree per iteration
        """
        self.bounds = bounds
        self.step_size = step_size
        self.nodes: List[RRTNode] = []

    def is_collision_free(self, config: np.ndarray) -> bool:
        """Check if configuration collides with obstacles."""
        # TODO: Implement collision checking with obstacle list
        # For now, simple bounds check
        return all(low <= val <= high for (low, high), val in zip(self.bounds, config))

    def sample_random_config(self) -> np.ndarray:
        """Sample random configuration in C-space."""
        return np.array([np.random.uniform(low, high) for low, high in self.bounds])

    def nearest_node(self, config: np.ndarray) -> RRTNode:
        """Find tree node closest to config."""
        distances = [np.linalg.norm(node.config - config) for node in self.nodes]
        return self.nodes[np.argmin(distances)]

    def steer(self, from_config: np.ndarray, to_config: np.ndarray) -> np.ndarray:
        """Move from from_config toward to_config by step_size."""
        direction = to_config - from_config
        distance = np.linalg.norm(direction)

        if distance <= self.step_size:
            return to_config
        else:
            return from_config + (direction / distance) * self.step_size

    def plan(self, start: np.ndarray, goal: np.ndarray, max_iterations: int = 5000, goal_tolerance: float = 0.5) -> List[np.ndarray]:
        """
        Plan path from start to goal using RRT.

        Returns:
            List of configurations from start to goal, or empty list if failed.
        """
        # Initialize tree
        self.nodes = [RRTNode(start)]

        for _ in range(max_iterations):
            # Sample random configuration (bias toward goal 10% of time)
            if np.random.rand() < 0.1:
                q_rand = goal
            else:
                q_rand = self.sample_random_config()

            # Find nearest node and steer toward sample
            q_near_node = self.nearest_node(q_rand)
            q_new = self.steer(q_near_node.config, q_rand)

            # Add to tree if collision-free
            if self.is_collision_free(q_new):
                q_new_node = RRTNode(q_new)
                q_new_node.parent = q_near_node
                self.nodes.append(q_new_node)

                # Check if goal reached
                if np.linalg.norm(q_new - goal) < goal_tolerance:
                    # Reconstruct path
                    path = [q_new]
                    current = q_new_node
                    while current.parent is not None:
                        current = current.parent
                        path.append(current.config)
                    path.reverse()
                    return path

        # Failed to find path
        return []


# Example usage
if __name__ == '__main__':
    # 2D planning problem
    bounds = [(0, 10), (0, 10)]  # x, y ranges
    planner = RRTPlanner(bounds, step_size=0.5)

    start = np.array([0.0, 0.0])
    goal = np.array([9.0, 9.0])

    path = planner.plan(start, goal, max_iterations=1000)

    if path:
        print(f"Path found with {len(path)} waypoints")
    else:
        print("No path found within iteration limit")
```

### RRT Variants

**RRT*:**
- Rewires tree to optimize path cost
- Asymptotically optimal (path improves with time)
- Slower than RRT

**RRT-Connect:**
- Grows two trees (from start and goal)
- Faster convergence
- Good for narrow passages

**Informed RRT*:**
- Samples only in ellipse between start and goal
- Faster optimization

## Kinodynamic Planning

**Challenge:** Real robots have dynamics (velocity, acceleration limits).

**Problem:** A* path might have sharp turns that robot can't execute.

### Velocity Constraints

**Humanoid walking:**
- Max velocity: 1.0 m/s
- Max angular velocity: 0.5 rad/s
- Acceleration limits

**Solution:** Plan in state space (x, y, θ, v, ω) instead of just (x, y, θ).

### Dynamic Window Approach (DWA)

**Idea:** Consider only velocities reachable in next time step.

**Algorithm:**
1. Generate velocity samples within dynamic window
2. Simulate forward for each velocity
3. Score trajectories: `score = α*heading + β*dist + γ*velocity`
4. Select best trajectory

**Used in:** Nav2 local planner (covered in exercise)

## Comparison of Planning Methods

| Method | Type | Optimal | Speed | High-Dim | Dynamic |
|--------|------|---------|-------|----------|---------|
| **Dijkstra** | Grid | Yes | Slow | No | No |
| **A*** | Grid | Yes | Fast | No | No |
| **RRT** | Sampling | No | Fast | Yes | No |
| **RRT*** | Sampling | Asymptotic | Medium | Yes | No |
| **DWA** | Local | No | Fast | No | Yes |

### When to Use Each

**A* (Grid-based):**
- Static environments
- 2D/3D spaces
- Need optimal paths
- Example: Indoor robot navigation

**RRT (Sampling-based):**
- High-dimensional spaces (7+ DOF)
- Complex obstacles
- Robot arms, humanoid full-body
- Example: Whole-body motion planning

**DWA (Reactive):**
- Dynamic obstacles
- Real-time execution
- Local obstacle avoidance
- Example: Humanoid walking with pedestrians

## Replanning and Reactive Avoidance

### When to Replan

**Triggers:**
- New obstacle detected (sensor data)
- Goal changed (task update)
- Path blocked (collision imminent)

### D* Lite Algorithm

**Incremental replanning:**
- Reuses previous search results
- Only updates affected cells
- Much faster than replanning from scratch

**Used in:** Mars rovers, dynamic environments

### Potential Fields (Reactive)

**Idea:** Goal attracts, obstacles repel.

**Forces:**
```python
F_attractive = k_att * (goal - robot)
F_repulsive = k_rep * sum(1/distance^2 for obstacle in nearby)
F_total = F_attractive + F_repulsive
```

**Pros:**
- Simple, fast
- Reactive (no planning)

**Cons:**
- Local minima (robot gets stuck)
- Not optimal

## Key Takeaways

- **Planning finds collision-free paths from start to goal**
- **A* is optimal and fast for grid-based 2D/3D navigation**
- **Heuristics guide search: Euclidean, Manhattan, Octile**
- **RRT handles high-dimensional spaces and complex obstacles**
- **Kinodynamic planning respects robot dynamics**
- **Grid methods: optimal but memory-intensive**
- **Sampling methods: fast for high dimensions but suboptimal**
- **Reactive methods (DWA, potential fields): handle dynamic obstacles**
- **Replanning (D* Lite): efficient updates when environment changes**

## Next Steps

In **[Lesson 2: Reinforcement Learning for Robots](lesson-02-reinforcement-learning.md)**, you will:
- Formulate robot tasks as Markov Decision Processes (MDPs)
- Implement policy gradient methods (PPO) for humanoid balance
- Train DQN agents for navigation tasks in Gazebo
- Apply sim-to-real transfer techniques
- Use stable-baselines3 for RL training

**Before moving on**, make sure you can:
- Explain the difference between grid-based and sampling-based planning
- Implement A* pathfinding with proper heuristics
- Understand when RRT is better than A*
- Recognize kinodynamic constraints in robot motion

---

**Estimated time for Lesson 2:** 3 hours (RL concepts + training)
