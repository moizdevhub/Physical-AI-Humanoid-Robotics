---
id: project-scenarios
title: Project Description & Scenarios
sidebar_position: 2
---

# Capstone Project: Scenarios & Examples

## Overview

This guide presents **detailed project scenarios** to inspire your capstone. Each scenario demonstrates how to integrate all four modules into a cohesive humanoid robot system. You can implement one of these scenarios exactly, modify them to suit your interests, or design an entirely custom project.

**Key principle:** Your project must meaningfully integrate **all four modules** — not just include them superficially.

## Scenario 1: Kitchen Assistant Robot

**Complexity:** Medium
**Estimated effort:** 25-30 hours
**Demo task:** "Clean the kitchen table"

### Scenario Description

A humanoid robot assists with kitchen cleanup by identifying objects on a table and moving them to appropriate locations (sink for dishes, drawer for utensils, trash for waste).

### User Interaction

**Command:** `"Clean the kitchen table"`

**Robot behavior:**
1. Navigate to kitchen table
2. Detect all objects on table (plates, cups, utensils)
3. For each object:
   - Classify object type (dish, utensil, trash)
   - Grasp object
   - Navigate to destination (sink, drawer, trash bin)
   - Place object
4. Return to home position
5. Report: "Kitchen table cleaned. Moved 3 dishes to sink, 2 forks to drawer."

### Module Integration

**Module 1 (ROS 2):**
- Action server for multi-step task execution
- Service for object classification queries
- Topic for status updates (`/robot/status`)
- Parameter server for location mappings (table → sink coordinates)

**Module 2 (Digital Twin):**
- Gazebo world with kitchen environment (table, sink, drawer, trash)
- URDF humanoid with mobile base, 7-DOF arm, 2-finger gripper
- Camera mounted on head, depth sensor on torso
- Friction parameters tuned for realistic object grasping

**Module 3 (AI Brain):**
- Nav2 for navigation between table, sink, drawer, trash
- Behavior tree: `Sequence[Detect Objects, For Each Object: Classify, Grasp, Navigate, Place]`
- A* global planner for collision-free paths
- Replanning when path blocked

**Module 4 (VLA):**
- YOLO for object detection (detects plates, cups, forks)
- CLIP for object classification ("plate" → sink, "fork" → drawer)
- Grasp planner computes 6D poses from bounding boxes
- Depth sensor provides 3D positions

### Acceptance Criteria

- [ ] Robot navigates to table without collision
- [ ] Detects at least 3 different object types
- [ ] Successfully grasps and transports at least 2 objects
- [ ] Places objects in correct destinations (dishes → sink)
- [ ] Handles failure gracefully (e.g., object too heavy, path blocked)
- [ ] RViz2 visualization shows detection bounding boxes and planned paths

### Extension Ideas

- **Hard:** Add human detection — pause task if human approaches
- **Hard:** Train RL policy for robust grasping (instead of analytic grasp planner)
- **Medium:** Voice input using speech-to-text (Whisper)
- **Easy:** Add sound effects for task completion

---

## Scenario 2: Object Sorting & Rearrangement

**Complexity:** Medium
**Estimated effort:** 20-25 hours
**Demo task:** "Sort the blocks by color"

### Scenario Description

A humanoid robot sorts colored blocks from a cluttered workspace into organized bins based on color or shape.

### User Interaction

**Command:** `"Sort all red blocks to the left bin"`

**Robot behavior:**
1. Scan workspace to detect all blocks
2. Filter detections: keep only red blocks
3. For each red block:
   - Plan grasp approach
   - Grasp block
   - Navigate to left bin
   - Place block in bin
4. Report: "Sorted 5 red blocks to left bin"

### Module Integration

**Module 1 (ROS 2):**
- Topic `/sorting_command` for text commands
- Topic `/block_detections` publishes detected blocks with colors
- Service `/plan_grasp` returns grasp pose for given block ID
- Action `/execute_sort` for full sort sequence

**Module 2 (Digital Twin):**
- Gazebo world with table, bins (left, center, right)
- Colored block objects (red, blue, green cubes)
- Randomized initial block positions each episode
- Overhead camera for workspace view

**Module 3 (AI Brain):**
- TAMP (Task and Motion Planning) to sequence pick-place operations
- Constraint: robot can only hold one block at a time
- Behavior tree handles errors (block slipped, bin full)
- RL-trained gripper policy for robust grasping (optional)

**Module 4 (VLA):**
- YOLO trained on colored blocks (custom dataset)
- CLIP grounds "red blocks" to matching detections
- Color classification with HSV thresholding (fallback)
- Depth map for 3D block positions

### Acceptance Criteria

- [ ] Detects all blocks with correct color labels
- [ ] Sorts at least 80% of blocks to correct bins
- [ ] No dropped blocks (grasp success >90%)
- [ ] Handles occlusions (blocks stacked or overlapping)
- [ ] Completes sorting in less than 5 minutes simulation time

### Extension Ideas

- **Hard:** Multi-robot collaboration (two robots sorting together)
- **Hard:** Sort by shape (cube, cylinder, pyramid) using 3D segmentation
- **Medium:** Active perception — move camera to see occluded blocks
- **Easy:** Count blocks and report statistics

---

## Scenario 3: Tool Fetching Assistant

**Complexity:** High
**Estimated effort:** 30-35 hours
**Demo task:** "Bring me the wrench"

### Scenario Description

A humanoid robot navigates a workshop environment, finds requested tools, and delivers them to the user.

### User Interaction

**Command:** `"Bring me the wrench from the toolbox"`

**Robot behavior:**
1. Parse command: tool = "wrench", location = "toolbox"
2. Navigate to toolbox
3. Open toolbox lid (manipulation challenge)
4. Search for wrench inside (active perception)
5. Grasp wrench
6. Close toolbox lid
7. Navigate to user location
8. Hand over wrench
9. Report: "Delivered wrench"

### Module Integration

**Module 1 (ROS 2):**
- Topic `/voice_command` receives text commands
- Service `/parse_command` extracts entities (tool, location)
- Action `/fetch_object` full fetch-and-deliver sequence
- TF tree for object locations (toolbox, user, charging station)

**Module 2 (Digital Twin):**
- Workshop environment: toolbox, shelves, user position marker
- Articulated toolbox with hinged lid
- Multiple tool objects (wrench, hammer, screwdriver)
- Realistic contact physics for lid manipulation

**Module 3 (AI Brain):**
- PDDLStream for task planning (navigate, open, grasp, close, deliver)
- Nav2 with obstacle avoidance (dynamic costmap updates)
- Behavior tree: `Sequence[Parse → Navigate → Open → Search → Grasp → Close → Deliver]`
- Error recovery: if tool not found, explore toolbox

**Module 4 (VLA):**
- YOLO fine-tuned on tools dataset
- CLIP for tool name grounding ("wrench" → bounding box)
- VLA model (RT-1 style) for lid manipulation policy
- Active exploration: move head camera to search inside toolbox

### Acceptance Criteria

- [ ] Successfully navigates to toolbox in cluttered environment
- [ ] Opens toolbox lid without collision
- [ ] Detects requested tool among other objects
- [ ] Grasps tool securely
- [ ] Delivers tool to user position
- [ ] Closes toolbox lid after retrieval
- [ ] Handles "tool not found" gracefully (reports failure, suggests alternatives)

### Extension Ideas

- **Hard:** Multi-object requests ("bring wrench and hammer")
- **Hard:** Train VLA end-to-end (language → actions) instead of modular pipeline
- **Medium:** Natural language parsing with LLM (GPT-4 via API)
- **Medium:** Return tool to toolbox after use

---

## Scenario 4: Follow-Me Companion

**Complexity:** Medium-High
**Estimated effort:** 25-30 hours
**Demo task:** "Follow me around the room"

### Scenario Description

A humanoid robot follows a person at a safe distance, maintaining visual contact and avoiding obstacles.

### User Interaction

**Command:** `"Follow me"` (or detected via gesture recognition)

**Robot behavior:**
1. Detect person in camera view
2. Estimate person's position in 3D
3. Plan path to position 1.5m behind person
4. Execute motion while tracking person
5. If person stops, robot stops
6. If person lost, rotate to search
7. If person says "stop following," return to home

### Module Integration

**Module 1 (ROS 2):**
- Topic `/person_pose` publishes detected person 3D position
- Topic `/follow_command` start/stop following
- Service `/get_safe_distance` computes target position behind person
- Dynamic TF frame `person_frame` updated in real-time

**Module 2 (Digital Twin):**
- Indoor environment (hallway, rooms, furniture)
- Human model that can be teleoperated or scripted
- LiDAR for obstacle avoidance
- Camera for person detection

**Module 3 (AI Brain):**
- Dynamic Window Approach (DWA) for reactive navigation
- Behavior tree: `Selector[Person Visible: Follow, Person Lost: Search]`
- Replanning at 10 Hz for real-time tracking
- Safe distance maintenance (1.5m, configurable)

**Module 4 (VLA):**
- MediaPipe for human pose detection (33 keypoints)
- Person re-identification to handle occlusions
- Depth estimation for 3D person position
- Gesture recognition to start/stop following (wave = start, palm = stop)

### Acceptance Criteria

- [ ] Detects person and starts following on command
- [ ] Maintains safe distance (1.0-2.0m)
- [ ] Follows person through multiple rooms
- [ ] Avoids obstacles while following (furniture, walls)
- [ ] Stops when person stops
- [ ] Recovers if person temporarily lost (searches, re-acquires)
- [ ] Responds to stop command (voice or gesture)

### Extension Ideas

- **Hard:** Multi-person scenario (follow specific person by shirt color)
- **Hard:** Predict person trajectory and plan proactive path
- **Medium:** Carry objects while following (tray-carrying robot)
- **Easy:** Play sound when starting/stopping following

---

## Scenario 5: Custom Project

**Complexity:** Variable
**Estimated effort:** 20-40 hours depending on scope

### Design Your Own

If the above scenarios don't match your interests, design a custom project following these guidelines:

**Required Elements:**
1. **Natural language input** (text or voice command)
2. **Vision-based perception** (detect/classify objects or humans)
3. **Autonomous navigation** (move from A to B, avoiding obstacles)
4. **Manipulation or interaction** (grasp object, press button, open door)
5. **Multi-step task** (at least 3 sequential actions)
6. **Error handling** (gracefully handle at least 2 failure modes)

**Example Custom Ideas:**
- **Home Security:** Patrol house, detect intruders with person detection, sound alarm
- **Library Assistant:** Find book by title on shelf, retrieve and deliver
- **Waiter Robot:** Take order via voice, fetch drink from fridge, deliver to table
- **Gardening Robot:** Detect wilted plants, navigate to plant, water it
- **Laundry Folder:** Detect clothing items, classify type, fold and stack

**Approval Process:**
Before implementing a custom project, validate your idea ensures integration of all four modules:

| Module | Your Integration | Meaningful? |
|--------|------------------|-------------|
| ROS 2 | _Description_ | Yes/No |
| Digital Twin | _Description_ | Yes/No |
| AI Brain | _Description_ | Yes/No |
| VLA | _Description_ | Yes/No |

**If any module is "No"**, revise your scenario to include it meaningfully.

---

## Integration Anti-Patterns (What NOT to Do)

### ❌ Superficial Integration

**Bad Example:**
- ROS 2: "I have a ROS 2 workspace" (just structure, no custom nodes)
- Digital Twin: "Robot spawned in Gazebo" (but all logic runs outside simulation)
- AI Brain: "I use A* somewhere" (unconnected to actual navigation)
- VLA: "I loaded YOLO" (but don't use detections for anything)

**Why it's bad:** Modules exist but don't interact or contribute to task.

### ✅ Meaningful Integration

**Good Example:**
- ROS 2: Custom action server orchestrates task, services query planners
- Digital Twin: Gazebo physics validates grasps, camera feed streams to VLA
- AI Brain: Nav2 plans collision-free paths executed by robot, BT handles failures
- VLA: YOLO detections drive decision-making, CLIP grounds language to actions

**Why it's good:** Each module has a clear role in achieving the end-to-end task.

---

## Project Scoping Advice

### Too Ambitious (Likely to fail)

- Humanoid robot that cooks a full meal (10+ manipulation steps)
- Multi-robot soccer game (complex coordination, physics tuning)
- Autonomous warehouse with 50+ objects (data collection, training)

**Problem:** Scope exceeds 30-hour time budget. Focus on depth, not breadth.

### Well-Scoped (Achievable)

- Humanoid robot that sets a table (3 objects: plate, cup, fork)
- Two robots playing catch (grasp, throw, catch sequence)
- Warehouse robot that sorts 5 objects into 3 bins

**Why it works:** Clear success criteria, manageable complexity, all modules integrated.

### Too Simple (Insufficient)

- Robot that moves to fixed position and picks up one object
- YOLO detection that publishes bounding boxes (no action taken)
- Navigation demo with no manipulation or vision

**Problem:** Doesn't demonstrate integration or challenge student skills.

---

## Choosing Your Scenario

**Ask yourself:**

1. **Interest:** Am I excited about this problem? (Motivation matters for 30 hours of work)
2. **Complexity:** Can I complete this in 2 weeks? (Be realistic about time)
3. **Integration:** Does each module contribute meaningfully? (Not just checked off)
4. **Demo:** Can I show end-to-end success in a 3-minute video? (Avoid 20-minute demos)

**Start simple, add extensions:** It's better to have a simple system that works perfectly than an ambitious system that fails. Implement the core scenario first, then add bonus features if time permits.

---

## Next Steps

1. **Choose a scenario** (or design your own)
2. **Sketch architecture** (ROS 2 node graph, see Requirements & Rubric)
3. **List integration points** (how modules connect)
4. **Create milestone plan** (Week 13 vs Week 14 goals)
5. **Begin implementation** (start with Module 1+2 integration)

**Good luck building your integrated humanoid robot system!**
