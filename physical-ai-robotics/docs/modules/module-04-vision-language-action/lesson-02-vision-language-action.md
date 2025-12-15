---
id: lesson-02-vision-language-action
title: Lesson 2 - Vision-Language-Action Models
sidebar_position: 3
---

# Lesson 2: Vision-Language-Action Models

## Introduction

**Vision-Language-Action (VLA) models** are the frontier of embodied AI, enabling robots to understand natural language instructions and execute them through physical actions. Unlike traditional robotics that requires hand-coded policies for each task, VLA models learn end-to-end mappings from multimodal inputs (images + language) to robot actions.

In this lesson, you'll understand VLA architecture, explore state-of-the-art models (RT-1, RT-2, Octo, OpenVLA), learn about imitation learning and behavior cloning, and discover how to train and deploy VLA models on humanoid robots.

## The VLA Problem

### Traditional vs. VLA Approach

**Traditional robotics pipeline:**
```
Perception → Task Planning → Motion Planning → Control
     ↓            ↓               ↓              ↓
  YOLO        A* search       RRT/MoveIt    PID/MPC
```
- Each module hand-engineered
- Brittle to edge cases
- Hard to generalize to new tasks

**VLA approach:**
```
[Image, Language] → Neural Network → Action
     ↓                    ↓               ↓
  "Pick red cube"      RT-1/RT-2      [gripper_pos, gripper_cmd]
```
- End-to-end learning
- Generalizes from demonstrations
- Scales with data and model size

### Key Insight: Actions as Language

**VLA models treat actions as tokens** (like words in language models).

**Example:**
```
Input:  [Image of table, "pick up the cup"]
Output: [move_left, move_forward, open_gripper, move_down, close_gripper]
```

This allows:
- **Transformer architectures:** Same attention mechanism as GPT
- **Sequence modeling:** Actions predicted autoregressively
- **Pretraining:** Language models transfer to robotics

## VLA Architecture

### Core Components

**1. Vision Encoder**
- Processes RGB image(s) from robot camera
- Extracts visual features (typically 512-1024 dim)
- Architecture: ResNet, Vision Transformer (ViT), EfficientNet

**2. Language Encoder**
- Encodes natural language instruction
- Converts text to embedding (768 dim for BERT, 4096 dim for LLaMA)
- Architecture: BERT, T5, CLIP text encoder, pretrained LLMs

**3. Fusion Module**
- Combines vision + language features
- Cross-attention or concatenation
- Produces unified representation

**4. Action Decoder**
- Predicts robot actions from fused features
- Output: Continuous actions (joint positions/velocities) or discrete action tokens
- Architecture: MLP, Transformer decoder, or diffusion model

### High-Level Flow

```python
# Pseudocode for VLA inference
def vla_inference(image, text_instruction):
    # 1. Vision encoding
    visual_features = vision_encoder(image)  # (batch, 196, 768) for ViT

    # 2. Language encoding
    text_tokens = tokenizer(text_instruction)
    language_features = language_encoder(text_tokens)  # (batch, seq_len, 768)

    # 3. Fusion
    fused_features = cross_attention(visual_features, language_features)

    # 4. Action decoding
    action = action_decoder(fused_features)  # (batch, action_dim)

    return action
```

## RT-1: Robotics Transformer 1

**First large-scale VLA model from Google DeepMind (2022).**

### Architecture

**Vision:** EfficientNet-B3 (pretrained on ImageNet)
**Language:** Universal Sentence Encoder (USE)
**Fusion:** FiLM (Feature-wise Linear Modulation)
**Action Decoder:** Transformer with token learning

**Action tokenization:**
- Discretize continuous actions into 256 bins per dimension
- Action space: `[x, y, z, roll, pitch, yaw, gripper_open]` (7D)
- Total tokens: 256^7 (huge space!)
- **Solution:** Predict each dimension independently

### Training Data

**RT-1 dataset:**
- 130,000 episodes
- 700+ tasks ("pick X", "move Y to Z", "open drawer")
- Collected from multiple robot embodiments
- Real-world demonstrations (no simulation)

### Behavior Cloning

**RT-1 uses behavior cloning** (supervised imitation learning).

**Training objective:**
```
Minimize: L = -log P(action_t | observation_t, instruction)
```

**Process:**
1. Human teleoperates robot to complete task
2. Record (image, language, action) tuples
3. Train model to predict expert actions

**Pseudocode:**
```python
# Behavior cloning training loop
for episode in dataset:
    for timestep in episode:
        image = timestep.image
        instruction = episode.language_instruction
        action_gt = timestep.action  # Ground truth from demo

        # Forward pass
        action_pred = model(image, instruction)

        # Loss: MSE for continuous actions
        loss = mse_loss(action_pred, action_gt)

        # Backward pass
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()
```

**Pros:**
- Simple to implement
- Stable training
- Works with limited data

**Cons:**
- No exploration (only copies demonstrations)
- Distribution shift (robot deviates from demos)
- Can't improve beyond human demonstrator

### RT-1 Results

**Generalization:**
- 97% success on trained tasks
- 76% success on novel objects
- 52% success on novel environments

**Key finding:** Scaling data improves generalization significantly.

## RT-2: Vision-Language-Action via VLM

**RT-2 (2023): Leverages pretrained vision-language models.**

### Key Innovation: Web-Scale Pretraining

**Problem:** RT-1 trained only on robot data (130K episodes).
**Insight:** Vision-language models (VLMs) trained on billions of web images understand the visual world.

**RT-2 approach:**
1. Start with pretrained VLM (PaLI-X or PaLM-E)
2. Add action tokens to vocabulary
3. Fine-tune on robot demonstrations

**Architecture:**
- **Vision:** ViT-22B (22 billion parameters)
- **Language:** PaLM-E (562B parameters)
- **Action prediction:** Text decoder generates action tokens

### Action as Text

**RT-2 treats actions as language tokens.**

**Example:**
```
Input:  [Image, "pick up the can"]
Output: "action: [0.12, -0.05, 0.30, 0.0, 1.57, 0.0, 1]"
```

**Benefits:**
- Leverage language model pretraining
- No separate action decoder needed
- Can generate both actions and text explanations

### RT-2 Results

**Performance improvements over RT-1:**
- **Novel objects:** 76% → 85%
- **Novel environments:** 52% → 71%
- **Emergent capabilities:** Can follow complex instructions like "move banana to the sum of two plus one" (interprets "sum of two plus one" = 3, moves to position 3)

**Key finding:** VLM pretraining provides semantic understanding beyond robot data.

## Octo: Open-Source Generalist Robot Policy

**Octo (2024): Open-source VLA model trained on diverse robot data.**

### Open X-Embodiment Dataset

**Largest open robot dataset:**
- 1 million+ trajectories
- 22 robot embodiments (arms, mobile manipulators, humanoids)
- Diverse tasks and environments
- **Goal:** Train generalist policy that works across robots

### Architecture

**Vision:** ViT-B/16 (pretrained on ImageNet)
**Language:** T5-XXL text encoder
**Action Decoder:** Diffusion policy

**Key innovation: Diffusion for actions**
- Actions as denoising diffusion process
- Better handles multimodal action distributions
- More robust than deterministic prediction

**Diffusion policy:**
```python
# Training: Add noise to actions
def diffusion_training_step(actions_gt):
    # Sample noise level
    t = np.random.randint(0, T)  # T=100 diffusion steps

    # Add noise to actions
    noise = np.random.randn(*actions_gt.shape)
    actions_noisy = sqrt(alpha_t) * actions_gt + sqrt(1 - alpha_t) * noise

    # Predict noise
    noise_pred = model(actions_noisy, t, observation)

    # Loss: predict noise
    loss = mse_loss(noise_pred, noise)
    return loss

# Inference: Denoise from random noise
def diffusion_inference(observation):
    # Start from random noise
    actions = np.random.randn(action_dim)

    # Iterative denoising
    for t in reversed(range(T)):
        noise_pred = model(actions, t, observation)
        actions = denoise_step(actions, noise_pred, t)

    return actions
```

### Cross-Embodiment Transfer

**Challenge:** Different robots have different action spaces.

**Octo solution:**
- **Action tokenization:** Normalize actions to [-1, 1] range
- **Observation adapters:** Convert robot-specific observations to common format
- **Fine-tuning:** Adapt to new robot with small amount of data

**Example:**
```python
# Train on Franka Panda (7 DOF arm)
actions_franka = [j1, j2, j3, j4, j5, j6, j7, gripper]  # 8D

# Fine-tune on UR5 (6 DOF arm)
actions_ur5 = [j1, j2, j3, j4, j5, j6, gripper]  # 7D

# Octo learns to map both to same latent action space
```

### Octo Results

**Zero-shot transfer:**
- 65% success on unseen robot after 50 demos
- 85% after 200 demos

**Task generalization:**
- Trained on pick-and-place → generalizes to stacking
- Trained on drawer opening → generalizes to door opening

## OpenVLA: Open-Source VLA Platform

**OpenVLA (2024): Open implementation of RT-2 style models.**

### Architecture

**Vision:** SigLIP (better than CLIP for robotics)
**Language:** Llama 2 (7B parameters)
**Training:** LoRA fine-tuning for efficiency

**Key features:**
- Fully open-source (Apache 2.0 license)
- Efficient training (single A100 GPU)
- Works with custom robot datasets

### Training Pipeline

```python
from openvla import VLAModel, VLATrainer

# Load pretrained vision-language model
model = VLAModel.from_pretrained("siglip-llama-7b")

# Prepare dataset
dataset = load_robot_dataset(
    data_dir="/path/to/demos",
    format="rlds"  # TensorFlow Datasets format
)

# Fine-tune on robot data
trainer = VLATrainer(
    model=model,
    dataset=dataset,
    learning_rate=1e-5,
    batch_size=16,
    num_epochs=10
)

trainer.train()

# Save fine-tuned model
model.save("humanoid_vla.pth")
```

### Inference

```python
# Load trained model
model = VLAModel.from_pretrained("humanoid_vla.pth")

# Run inference
observation = {
    'image': robot_camera_image,  # (224, 224, 3)
    'proprio': robot_joint_state,  # (num_joints,)
}
instruction = "pick up the red cube"

action = model.predict(observation, instruction)
# Returns: (action_dim,) array
```

## Behavior Cloning vs. Imitation Learning

### Behavior Cloning (BC)

**Supervised learning from expert demonstrations.**

**Pros:**
- Simple: minimize prediction error
- Stable training
- Works with offline data

**Cons:**
- Compounding errors (distribution shift)
- Can't correct mistakes
- Limited to expert performance

### DAgger (Dataset Aggregation)

**Addresses distribution shift in BC.**

**Algorithm:**
1. Train policy π from expert demos
2. Deploy π, collect on-policy data
3. Query expert for correct actions
4. Aggregate new data with original dataset
5. Retrain policy
6. Repeat

**Benefits:**
- Reduces distribution shift
- Policy learns to recover from mistakes

**Challenge:** Requires expert available during training (online learning).

### Inverse Reinforcement Learning (IRL)

**Learn reward function from demonstrations.**

**Idea:** Expert is implicitly optimizing some reward function. Infer that reward, then optimize it.

**Process:**
1. Observe expert demonstrations
2. Learn reward function R that expert maximizes
3. Use RL (PPO, SAC) to optimize learned reward

**Pros:**
- More robust than BC
- Can outperform demonstrator
- Generalizes better

**Cons:**
- Complex to implement
- Requires solving RL problem (slow)

### Comparison

| Method | Data Efficiency | Generalization | Performance Ceiling |
|--------|-----------------|----------------|---------------------|
| **Behavior Cloning** | High | Medium | Expert level |
| **DAgger** | Medium | High | Expert level |
| **IRL** | Low | High | Beyond expert |
| **Offline RL** | High | Medium | Beyond expert |

**For VLA models:** Behavior cloning is standard due to large datasets and simplicity.

## Action Tokenization Strategies

### Continuous Actions

**Direct regression:**
```python
# Predict continuous action values
action_pred = model(observation)  # (7,) for 7-DOF arm
# Loss: MSE
loss = ((action_pred - action_gt)**2).mean()
```

**Pros:** Smooth actions
**Cons:** Struggles with multimodal distributions

### Discrete Actions (Binning)

**Discretize action space:**
```python
# RT-1 approach
num_bins = 256
action_bins = np.linspace(-1, 1, num_bins)

# Convert continuous to discrete
action_continuous = 0.42
action_discrete = np.argmin(np.abs(action_bins - action_continuous))

# Predict discrete token
action_logits = model(observation)  # (num_bins,)
action_discrete_pred = torch.argmax(action_logits)

# Convert back to continuous
action_pred = action_bins[action_discrete_pred]
```

**Pros:** Works with language model architectures
**Cons:** Quantization error, large token vocabulary

### Diffusion Actions

**Octo approach:**
- Model action distribution with diffusion process
- Handles multimodal distributions (e.g., two ways to grasp object)
- Smooth, high-quality actions

**Pros:** Best action quality
**Cons:** Slower inference (100 denoising steps)

## Training VLA Models

### Dataset Format

**Standard: RLDS (Reinforcement Learning Datasets)**

**Episode structure:**
```python
episode = {
    'steps': [
        {
            'observation': {
                'image': np.array(224, 224, 3),
                'proprio': np.array(7),  # Joint positions
            },
            'action': np.array(7),  # Joint velocities or positions
            'reward': 1.0,  # Optional
        },
        # ... more steps
    ],
    'language_instruction': "pick up the red cube"
}
```

### Data Augmentation

**Image augmentations:**
```python
import torchvision.transforms as T

augmentation = T.Compose([
    T.RandomCrop(224),
    T.RandomHorizontalFlip(p=0.5),
    T.ColorJitter(brightness=0.2, contrast=0.2),
    T.RandomErasing(p=0.1),  # Simulate occlusions
])

augmented_image = augmentation(image)
```

**Temporal augmentations:**
- Frame skipping (every 3rd frame)
- Time reversal (for symmetric tasks)

### Training Recipe

```python
# Typical VLA training setup
model = VLAModel(
    vision_encoder="efficientnet-b3",
    language_encoder="USE",
    action_dim=7,
    num_action_bins=256
)

optimizer = torch.optim.AdamW(model.parameters(), lr=1e-4)
scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=100_000)

for episode in dataset:
    for step in episode['steps']:
        # Get inputs
        image = step['observation']['image']
        instruction = episode['language_instruction']
        action_gt = step['action']

        # Forward pass
        action_pred = model(image, instruction)

        # Loss
        loss = F.cross_entropy(action_pred, action_gt)  # For discrete actions

        # Backward pass
        optimizer.zero_grad()
        loss.backward()
        torch.nn.utils.clip_grad_norm_(model.parameters(), 1.0)
        optimizer.step()
        scheduler.step()
```

**Hyperparameters:**
- Learning rate: 1e-4 to 1e-5
- Batch size: 256-512 (large for stability)
- Training time: 3-7 days on 8 GPUs

## Deploying VLA Models

### Model Optimization

**1. Quantization:**
```python
# PyTorch quantization
import torch.quantization

model_int8 = torch.quantization.quantize_dynamic(
    model, {torch.nn.Linear}, dtype=torch.qint8
)
# 4x smaller model, 2-3x faster inference
```

**2. ONNX export:**
```python
# Export to ONNX for optimized inference
torch.onnx.export(
    model,
    (dummy_image, dummy_text),
    "vla_model.onnx",
    opset_version=14
)
```

**3. TensorRT:**
```python
# NVIDIA TensorRT for GPU inference
import tensorrt as trt

# Convert ONNX to TensorRT engine
# Achieves 10-100x speedup on NVIDIA GPUs
```

### Real-Time Inference

**Target:** 10-20 Hz control loop

**Optimization strategies:**
- Use smaller vision backbones (MobileNet instead of ViT)
- Cache language embeddings (instruction doesn't change every frame)
- Asynchronous inference (GPU processes while robot moves)

```python
class RealTimeVLA:
    def __init__(self):
        self.model = load_model()
        self.language_cache = {}

    def predict(self, image, instruction):
        # Cache language encoding
        if instruction not in self.language_cache:
            self.language_cache[instruction] = self.model.encode_language(instruction)

        lang_embedding = self.language_cache[instruction]

        # Fast vision encoding
        vis_embedding = self.model.encode_vision(image)

        # Action prediction
        action = self.model.decode_action(vis_embedding, lang_embedding)

        return action
```

## Key Takeaways

- **VLA models map (vision, language) → actions end-to-end**
- **RT-1: First large-scale VLA with 130K robot demonstrations**
- **RT-2: Leverages vision-language pretraining for better generalization**
- **Octo: Open-source generalist policy trained on 1M+ trajectories**
- **OpenVLA: Efficient open-source implementation using LLaMA + SigLIP**
- **Behavior cloning is standard training method for VLA models**
- **Action tokenization: continuous, discrete, or diffusion**
- **Training requires large datasets (100K+ episodes for generalization)**
- **Deployment: quantization, ONNX, TensorRT for real-time inference**

## Next Steps

In **[Lesson 3: Manipulation Tasks](lesson-03-manipulation-tasks.md)**, you will:
- Understand grasp planning and execution
- Implement language-conditioned object rearrangement
- Learn Task and Motion Planning (TAMP)
- Handle error recovery and replanning
- Apply sim-to-real transfer for manipulation

**Before moving on**, make sure you can:
- Explain the difference between RT-1 and RT-2
- Understand behavior cloning training process
- Recognize when to use continuous vs. discrete action representations
- Appreciate the role of pretraining in VLA models

---

**Estimated time for Lesson 3:** 2 hours (manipulation concepts + TAMP)
