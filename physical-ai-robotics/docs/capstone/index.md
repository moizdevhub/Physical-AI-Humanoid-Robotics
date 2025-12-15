---
id: index
title: Capstone Project
sidebar_position: 5
---

# Capstone Project: Integrated Humanoid Robot System

## Project Overview

The **Capstone Project** synthesizes all four modules into a complete humanoid robot application demonstrating **embodied intelligence**. You'll build an end-to-end system that perceives the environment, understands natural language commands, plans actions, and executes them in simulation.

**This is the culmination of your learning journey** — transforming individual skills (ROS 2, simulation, AI, VLA) into a unified robotic system.

## Integration Requirement

Your capstone **must integrate all four modules**:

| Module | Integration Point |
|--------|------------------|
| **Module 1: ROS 2** | Communication infrastructure for sensors, planning, and control |
| **Module 2: Digital Twin** | Gazebo/Isaac Sim environment for testing and demonstration |
| **Module 3: AI Brain** | Path planning and decision-making algorithms |
| **Module 4: VLA** | Vision-language-action pipeline for perception and manipulation |

**Minimum viable integration:** Each module must contribute meaningfully to the final system. Mere presence of ROS 2 topics or a Gazebo window is insufficient — functionality must be demonstrated end-to-end.

## Example Scenario: "Robot, Pick Up the Red Cube"

Imagine a user gives the command: **"Pick up the red cube and place it on the table."**

Your integrated system responds:

### 1. **Perception (Module 4: VLA)**
- Humanoid robot's camera captures RGB image of environment
- YOLO detects all objects in scene
- CLIP grounds language ("red cube") to bounding box
- Depth estimation computes 3D position of target object

### 2. **Planning (Module 3: AI Brain)**
- Nav2 planner computes path from current position to object
- Grasp planner generates gripper approach trajectory
- Behavior tree orchestrates: Navigate → Grasp → Navigate → Place

### 3. **Execution (Module 1: ROS 2)**
- Action client sends navigation goal to Nav2 action server
- Joint trajectory controller executes arm motion
- Gripper actuator closes to grasp object
- All coordination happens via ROS 2 topics/services/actions

### 4. **Testing (Module 2: Digital Twin)**
- Entire sequence executes in Gazebo simulation
- Physics engine validates gripper contact and object dynamics
- Camera feed streams to VLA pipeline in real-time
- Success/failure logged for iterative improvement

## Project Requirements

### Functional Requirements

Your system must:

1. **Accept natural language commands** (text input via ROS 2 topic or CLI)
2. **Detect and localize objects** using vision models (YOLO/CLIP or similar)
3. **Plan collision-free paths** using Nav2 or custom planner
4. **Execute manipulation tasks** (grasp, place, push)
5. **Operate in Gazebo simulation** with realistic physics and sensors
6. **Provide real-time feedback** (RViz2 visualization, status messages)

### Technical Requirements

- **ROS 2 Humble** workspace with at least 3 custom packages
- **Gazebo simulation** with humanoid robot model (URDF)
- **AI integration** (at least one trained/pretrained model for vision or planning)
- **Language grounding** (CLIP, LLM, or rule-based NLP)
- **Error handling** (graceful failure if object not found, path blocked, etc.)

### Documentation Requirements

1. **README.md** with setup instructions (dependencies, build, launch)
2. **Architecture diagram** showing ROS 2 node graph and data flow
3. **Demo video** (3-5 minutes) showing end-to-end execution
4. **Reflection report** (2-3 pages) discussing challenges, design decisions, and future improvements

## Suggested Milestones (Weeks 13-14)

### Week 13: Integration & Testing

- **Day 1-2:** Create unified ROS 2 workspace, integrate Module 1-2 code
- **Day 3-4:** Connect vision pipeline (Module 4) to planning (Module 3)
- **Day 5-7:** End-to-end testing, debugging, iteration

### Week 14: Polish & Demonstration

- **Day 1-2:** Record demo video, capture failure cases
- **Day 3-4:** Write documentation (README, architecture diagram)
- **Day 5:** Final testing, reflection report
- **Day 6-7:** Submission and presentation

## Evaluation Criteria

Your capstone will be evaluated on:

- **Integration Completeness (40%)** — All four modules present and functional
- **System Robustness (25%)** — Handles edge cases (object not found, path blocked)
- **Code Quality (15%)** — Clean architecture, modularity, documentation
- **Demonstration (15%)** — Video clearly shows end-to-end capability
- **Reflection (5%)** — Thoughtful analysis of design trade-offs

## Example Projects (Inspiration)

Past students have built:

- **"Kitchen Assistant"** — Humanoid navigates to sink, grasps cup, places in dishwasher
- **"Object Sorter"** — Detects colored blocks, sorts into bins by color
- **"Follow-Me Robot"** — Uses person detection + tracking to follow human around room
- **"Tool Fetcher"** — Responds to "Bring me the wrench" by finding and delivering tool

**Your project can be simpler or more ambitious** — focus on demonstrating integration over complexity.

## Resources & Support

- **Template Workspace:** Download starter ROS 2 workspace with basic integration structure (coming soon)
- **Sample URDF Models:** Humanoid robot models compatible with Gazebo (coming soon)
- **Troubleshooting Guide:** Common integration issues and fixes (coming soon)

## Detailed Pages (Coming Soon)

<!-- Future pages to be created:
- Project Description & Scenarios — Detailed examples and variations  
- Requirements & Rubric — Comprehensive grading criteria
- Submission Guidelines — How to package and submit your project
-->

---

**Ready to start planning?** Begin by sketching your system architecture and identifying which modules contribute to each component.

**Need inspiration?** Review the module exercise previews to see how components can connect together.

**Questions?** Refer to troubleshooting guides or reach out during office hours.

---

## Quick Start Checklist

- [ ] Review all four module exercises to understand available components
- [ ] Choose a project scenario (or design your own)
- [ ] Sketch system architecture (ROS 2 node graph)
- [ ] Set up unified ROS 2 workspace
- [ ] Integrate Module 1 + Module 2 (ROS 2 in Gazebo)
- [ ] Add Module 3 (planning/navigation)
- [ ] Add Module 4 (vision/language)
- [ ] Test end-to-end pipeline
- [ ] Record demo video
- [ ] Write documentation
- [ ] Submit project

**Estimated effort:** 25-30 hours over 2 weeks — plan accordingly!
