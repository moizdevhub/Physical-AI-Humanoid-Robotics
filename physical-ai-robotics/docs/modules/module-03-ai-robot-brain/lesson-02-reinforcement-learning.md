---
id: lesson-02-reinforcement-learning
title: Lesson 2 - Reinforcement Learning for Robots
sidebar_position: 3
---

# Lesson 2: Reinforcement Learning for Robots

## Introduction

**Reinforcement Learning (RL)** enables robots to learn control policies through trial and error, without explicit programming. Instead of hand-crafting controllers, you define a reward function and let the robot discover optimal behaviors through interaction with the environment.

For humanoid robots, RL has achieved breakthrough results in locomotion, manipulation, and navigation tasks that were previously impossible with traditional control methods.

In this lesson, you'll learn how to formulate robot tasks as MDPs, implement policy gradient and value-based methods, and train RL agents in Gazebo simulation.

## Why RL for Robotics?

### Challenges with Traditional Control

**Problem:** Humanoid walking is complex:
- 25+ joints with coupled dynamics
- Contact forces with ground
- Balance constraints
- Disturbance rejection

**Traditional approach:** Hand-tune PID gains for each joint → months of work, brittle to changes.

**RL approach:** Define reward (walk forward + stay upright), train policy → learns robust controller automatically.

### RL Success Stories

**Boston Dynamics Atlas backflip:**
- RL policy trained in simulation
- Transferred to real robot
- Achieves acrobatic maneuvers

**Google's robot grasping:**
- 14 robots learning 24/7 for 3 months
- Achieved 96% grasp success rate
- Generalized to novel objects

**OpenAI's Dactyl:**
- Humanoid hand solving Rubik's cube
- Trained entirely in simulation with domain randomization
- Transferred to real robot without fine-tuning

## Markov Decision Process (MDP)

RL formalizes robot tasks as **Markov Decision Processes**.

### MDP Components

**1. State Space (S)**
- Observable properties of the environment
- Humanoid example: Joint positions, velocities, IMU orientation

**2. Action Space (A)**
- Controls available to the robot
- Humanoid example: Joint torques or target positions

**3. Transition Dynamics (P)**
- Probability of next state given current state and action
- `P(s' | s, a)` - usually unknown (learned through interaction)

**4. Reward Function (R)**
- Scalar feedback for state-action pairs
- `r = R(s, a)` - designed by human

**5. Discount Factor (γ)**
- Importance of future rewards (0 < γ < 1)
- γ = 0.99 typical (future rewards matter)

### Humanoid Walking MDP

```python
# State: 37-dimensional vector
state = [
    joint_positions[25],      # Hip, knee, ankle angles
    joint_velocities[25],     # Joint angular velocities
    torso_orientation[4],     # Quaternion
    torso_linear_vel[3],      # x, y, z velocity
    torso_angular_vel[3],     # Roll, pitch, yaw rates
    foot_contacts[2],         # Left, right foot ground contact
]

# Action: 12-dimensional vector (leg joint torques)
action = [
    left_hip_roll_torque,
    left_hip_pitch_torque,
    left_knee_torque,
    left_ankle_pitch_torque,
    left_ankle_roll_torque,
    # ... same for right leg
]

# Reward function
reward = (
    1.0 * forward_velocity      # Encourage walking forward
    - 0.1 * abs(lateral_velocity)  # Penalize sideways drift
    - 0.01 * sum(torque^2)      # Penalize high torques (energy)
    + 1.0 if upright else -10.0  # Big penalty for falling
)
```

### Policy and Value Functions

**Policy (π):**
- Mapping from states to actions
- `a = π(s)` (deterministic)
- `a ~ π(a|s)` (stochastic)

**Value Function (V):**
- Expected cumulative reward from state s
- `V(s) = E[Σ γ^t * r_t | s_0 = s]`

**Q-Function (action-value):**
- Expected cumulative reward from state s taking action a
- `Q(s, a) = E[Σ γ^t * r_t | s_0 = s, a_0 = a]`

**Goal:** Find optimal policy π* that maximizes expected reward.

## Policy Gradient Methods

Policy gradient methods directly optimize the policy parameters.

### REINFORCE Algorithm

**Basic policy gradient algorithm.**

**Idea:** Adjust policy parameters to increase probability of actions that led to high rewards.

**Update rule:**
```
θ ← θ + α * ∇_θ log π_θ(a|s) * G_t
```
- θ: Policy parameters (neural network weights)
- α: Learning rate
- G_t: Return (cumulative discounted reward)

**Algorithm:**
1. Collect trajectory by running policy: `τ = (s_0, a_0, r_0, s_1, a_1, r_1, ...)`
2. Compute returns: `G_t = Σ_{k=t}^T γ^(k-t) * r_k`
3. Update policy: `θ ← θ + α * Σ_t ∇_θ log π_θ(a_t|s_t) * G_t`
4. Repeat

**Pros:**
- Simple
- Works for continuous actions
- Guaranteed convergence (local optimum)

**Cons:**
- High variance (many samples needed)
- Sample inefficient

### Actor-Critic Methods

**Idea:** Use value function (critic) to reduce variance.

**Components:**
- **Actor:** Policy π_θ(a|s) that selects actions
- **Critic:** Value function V_ϕ(s) that estimates expected return

**Advantage:**
```
A(s, a) = Q(s, a) - V(s)
```
- How much better is action a compared to average?
- Reduces variance in policy gradient

**Update rules:**
```
Actor:  θ ← θ + α_actor * ∇_θ log π_θ(a|s) * A(s, a)
Critic: ϕ ← ϕ - α_critic * ∇_ϕ (V_ϕ(s) - target)^2
```

### Proximal Policy Optimization (PPO)

**State-of-the-art policy gradient method.**

**Key innovation:** Limit policy updates to prevent catastrophic performance collapse.

**Clipped objective:**
```
L^CLIP(θ) = E[min(
    r_t(θ) * A_t,
    clip(r_t(θ), 1-ε, 1+ε) * A_t
)]

where r_t(θ) = π_θ(a_t|s_t) / π_θ_old(a_t|s_t)  # Probability ratio
```

**Properties:**
- Stable training (clips large policy updates)
- Sample efficient
- Works well for robotics

**Used in:** OpenAI's Dactyl, many humanoid locomotion tasks

### PPO Training Example

```python
#!/usr/bin/env python3
"""
Train humanoid robot to walk using PPO
"""
import gymnasium as gym
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv

# Custom Gazebo environment (see exercise for full implementation)
class HumanoidGazeboEnv(gym.Env):
    """Humanoid robot environment in Gazebo."""

    def __init__(self):
        super().__init__()
        # State: joint positions, velocities, IMU
        self.observation_space = gym.spaces.Box(
            low=-float('inf'), high=float('inf'), shape=(37,)
        )
        # Action: joint torques
        self.action_space = gym.spaces.Box(
            low=-100, high=100, shape=(12,)
        )

    def step(self, action):
        # Send joint commands to Gazebo
        self._send_joint_commands(action)

        # Wait for physics step
        self._wait_for_step()

        # Get new state from Gazebo
        obs = self._get_observation()

        # Compute reward
        reward = self._compute_reward()

        # Check if episode ended (fell over or timeout)
        terminated = self._check_termination()
        truncated = self._check_truncation()

        return obs, reward, terminated, truncated, {}

    def _compute_reward(self):
        """Reward function for walking."""
        # Get robot state
        vel_x = self.get_base_velocity()[0]
        upright = self.get_orientation()[2] > 0.9  # Z-axis pointing up
        torques = self.get_last_action()

        # Reward components
        reward = 0.0
        reward += 1.0 * vel_x                    # Walk forward
        reward += 1.0 if upright else -10.0      # Stay upright
        reward -= 0.01 * sum(t**2 for t in torques)  # Energy penalty

        return reward

    def reset(self, seed=None):
        # Reset Gazebo simulation
        self._reset_gazebo()
        obs = self._get_observation()
        return obs, {}


# Create vectorized environment
env = DummyVecEnv([lambda: HumanoidGazeboEnv()])

# Create PPO agent
model = PPO(
    policy="MlpPolicy",
    env=env,
    learning_rate=3e-4,
    n_steps=2048,        # Steps per update
    batch_size=64,
    n_epochs=10,         # Epochs per update
    gamma=0.99,          # Discount factor
    gae_lambda=0.95,     # GAE parameter
    clip_range=0.2,      # PPO clipping
    verbose=1,
    tensorboard_log="./logs/"
)

# Train for 1 million timesteps
model.learn(total_timesteps=1_000_000)

# Save trained policy
model.save("humanoid_walking_ppo")

# Test trained policy
obs = env.reset()
for _ in range(1000):
    action, _ = model.predict(obs, deterministic=True)
    obs, reward, done, info = env.step(action)
    if done:
        obs = env.reset()
```

**Training time:** ~6-12 hours on GPU for walking policy.

## Value-Based Methods

Value-based methods learn Q-function, then derive policy from it.

### Q-Learning

**Idea:** Learn action-value function, then act greedily.

**Update rule:**
```
Q(s, a) ← Q(s, a) + α * [r + γ * max_a' Q(s', a') - Q(s, a)]
```

**Policy:**
```
π(s) = argmax_a Q(s, a)  # Choose action with highest Q-value
```

**Limitation:** Only works for discrete actions (need to enumerate all actions).

### Deep Q-Network (DQN)

**Extension:** Use neural network to approximate Q-function.

**Q-network:**
```python
Q(s, a; θ) ≈ Q*(s, a)  # Neural network with parameters θ
```

**Key innovations:**
1. **Experience replay:** Store transitions in buffer, sample randomly
   - Breaks correlation between consecutive samples
   - More sample efficient

2. **Target network:** Separate network for computing targets
   - Stabilizes training
   - Updated slowly (τ = 0.005)

**DQN Algorithm:**
```
1. Initialize Q-network Q(s, a; θ) and target network Q(s, a; θ^-)
2. Initialize replay buffer D
3. For each episode:
   a. Observe state s
   b. Select action: a = argmax_a Q(s, a; θ) with ε-greedy exploration
   c. Execute action, observe reward r and next state s'
   d. Store transition (s, a, r, s') in D
   e. Sample minibatch from D
   f. Compute target: y = r + γ * max_a' Q(s', a'; θ^-)
   g. Update Q-network: θ ← θ - α * ∇_θ (Q(s, a; θ) - y)^2
   h. Every C steps: θ^- ← θ (update target network)
```

**Good for:** Discrete action spaces (navigation: forward, left, right, stop)

### Soft Actor-Critic (SAC)

**State-of-the-art for continuous control.**

**Key features:**
- Off-policy (sample efficient)
- Maximum entropy RL (explores effectively)
- Continuous actions (good for robotics)

**Components:**
- Actor: π_θ(a|s) (stochastic policy)
- Two critics: Q_ϕ1(s, a), Q_ϕ2(s, a) (reduces overestimation)
- Entropy temperature: α (controls exploration)

**Objective:**
```
maximize E[Σ_t r_t + α * H(π(·|s_t))]
```
- Maximize reward AND policy entropy (encourages exploration)

**Used in:** High-DoF manipulation, dexterous hand control

## Sim-to-Real Transfer

**Challenge:** Policy trained in simulation may fail on real robot.

### Domain Randomization

**Idea:** Train on wide distribution of simulations, policy learns robust features.

**Randomize:**

**Physics parameters:**
```python
# Randomize each episode
mass = np.random.uniform(0.8 * nominal, 1.2 * nominal)
friction = np.random.uniform(0.5, 1.5)
damping = np.random.uniform(0.8 * nominal, 1.2 * nominal)
```

**Visual appearance:**
```python
# Randomize lighting, textures, colors
light_intensity = np.random.uniform(0.5, 2.0)
floor_texture = random.choice(texture_library)
robot_color = np.random.rand(3)  # RGB
```

**Sensor noise:**
```python
# Add realistic noise to observations
imu_reading = true_imu + np.random.normal(0, 0.01)
joint_pos = true_pos + np.random.normal(0, 0.001)
```

**Latency:**
```python
# Simulate communication delays
action_delay = np.random.uniform(0.01, 0.05)  # 10-50ms
```

**Result:** Policy robust to sim-to-real differences.

### System Identification

**Alternative approach:** Measure real robot properties, update simulation to match.

**Steps:**
1. Execute known trajectories on real robot
2. Measure actual response (positions, forces)
3. Fit simulation parameters to match real data
4. Train policy in updated simulation

**Pros:**
- More accurate simulation
- Less randomization needed

**Cons:**
- Requires real robot access
- Time-consuming

### Residual Learning

**Idea:** Learn correction on top of sim-trained policy.

**Process:**
1. Train policy π_sim in simulation
2. Deploy on real robot, collect data
3. Train residual policy π_residual(s) that outputs action correction
4. Final action: `a = π_sim(s) + π_residual(s)`

**Pros:**
- Fast fine-tuning (few real-world samples)
- Retains sim-trained behaviors

## Training Best Practices

### Reward Shaping

**Problem:** Sparse rewards (only +1 at goal) → slow learning.

**Solution:** Add dense reward signals.

**Example (humanoid walking):**
```python
# Sparse (bad)
reward = 1.0 if reached_goal else 0.0

# Dense (good)
reward = (
    1.0 * forward_velocity           # Make progress
    - 0.5 * abs(lateral_velocity)    # Stay on path
    - 0.1 * sum(action^2)            # Smooth actions
    + 0.1 * height                   # Stay upright
    - 0.01 * sum(torque^2)           # Energy efficiency
)
```

**Caution:** Don't over-shape (reward hacking).

### Curriculum Learning

**Idea:** Start with easy tasks, gradually increase difficulty.

**Example (humanoid locomotion):**
1. **Stage 1:** Stand upright (no movement)
2. **Stage 2:** Walk slowly (0.5 m/s target)
3. **Stage 3:** Walk at normal speed (1.0 m/s)
4. **Stage 4:** Walk over obstacles
5. **Stage 5:** Walk on uneven terrain

**Implementation:**
```python
def get_difficulty_level(timestep):
    if timestep < 100_000:
        return 1  # Standing
    elif timestep < 300_000:
        return 2  # Slow walking
    elif timestep < 600_000:
        return 3  # Normal walking
    else:
        return 4  # Obstacles
```

### Hyperparameter Tuning

**Key hyperparameters:**

**PPO:**
- `learning_rate`: 3e-4 (typical)
- `n_steps`: 2048 (rollout length)
- `batch_size`: 64
- `n_epochs`: 10
- `clip_range`: 0.2

**SAC:**
- `learning_rate`: 3e-4
- `buffer_size`: 1_000_000
- `learning_starts`: 10_000
- `batch_size`: 256
- `tau`: 0.005 (target network update)

**Use Optuna for automated search:**
```python
import optuna

def objective(trial):
    lr = trial.suggest_loguniform('lr', 1e-5, 1e-3)
    model = PPO("MlpPolicy", env, learning_rate=lr)
    model.learn(100_000)
    return evaluate_policy(model, env)

study = optuna.create_study(direction='maximize')
study.optimize(objective, n_trials=50)
```

## Comparison of RL Methods

| Method | Type | Sample Efficiency | Stability | Action Space | Robotics Use |
|--------|------|-------------------|-----------|--------------|--------------|
| **REINFORCE** | Policy Gradient | Low | Medium | Continuous | Simple tasks |
| **PPO** | Policy Gradient | Medium | High | Continuous | Locomotion, manipulation |
| **DQN** | Value-based | Medium | High | Discrete | Navigation |
| **SAC** | Actor-Critic | High | High | Continuous | Dexterous control |

### Recommendations for Robotics

**Locomotion (humanoid walking):**
- Use PPO
- Train 1-5M timesteps
- Domain randomization essential

**Manipulation (grasping, reaching):**
- Use SAC
- Train 500K-2M timesteps
- Careful reward shaping

**Navigation (discrete actions):**
- Use DQN
- Train 1-3M timesteps
- Image-based observations

## Key Takeaways

- **RL enables robots to learn policies through trial and error**
- **MDPs formalize robot tasks: state, action, reward, transition**
- **Policy gradient (PPO): directly optimize policy, stable, good for locomotion**
- **Value-based (DQN): learn Q-function, sample efficient, discrete actions**
- **Actor-Critic (SAC): combines policy and value, state-of-the-art continuous control**
- **Sim-to-real gap bridged by domain randomization and system ID**
- **Reward shaping and curriculum learning accelerate training**
- **stable-baselines3 provides production-ready implementations**

## Next Steps

In **[Lesson 3: Behavior Trees](lesson-03-behavior-trees.md)**, you will:
- Understand hierarchical task decomposition with behavior trees
- Learn node types: Sequence, Selector, Parallel, Decorator
- Implement reactive task execution with interruption
- Integrate behavior trees with ROS 2 action servers
- Compare behavior trees to finite state machines

**Before moving on**, make sure you can:
- Formulate a robot task as an MDP
- Explain the difference between policy gradient and value-based methods
- Understand when to use PPO vs SAC vs DQN
- Recognize domain randomization strategies for sim-to-real transfer

---

**Estimated time for Lesson 3:** 2 hours (behavior tree concepts + implementation)
