# Tasks: Physical AI & Humanoid Robotics Education Book

**Input**: Design documents from `/specs/001-robotics-education-book/`
**Prerequisites**: plan.md (required), spec.md (required for user stories)

**Tests**: No automated tests - content validation done manually during Phase 5 (Polish & Deployment)

**Organization**: Tasks are grouped by implementation phase and user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3, US4)
- Include exact file paths in descriptions

## Path Conventions

- **Docusaurus project**: All content in `/docs/`, static assets in `/static/`, config at root
- All MDX files created under `/docs/` directory
- All diagrams saved to `/static/img/modules/<module-name>/`
- All code examples saved to `/static/code/<module-name>/`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Initialize Docusaurus project and configure for educational content deployment

**Checkpoint**: Empty site deploys to GitHub Pages successfully

- [ ] T001 [US1] Initialize Docusaurus project with `npx create-docusaurus@latest physical-ai-robotics classic --typescript` at repository root
  - **Acceptance**: `package.json` exists with Docusaurus 3.x dependency, `npm start` launches dev server at localhost:3000

- [ ] T002 [US1] Configure `docusaurus.config.js` with site metadata (title: "Physical AI & Humanoid Robotics", tagline, base URL for GitHub Pages)
  - **Acceptance**: Site displays correct title and tagline when running `npm start`

- [ ] T003 [US1] Configure navbar in `docusaurus.config.js` with links to Modules dropdown, Capstone, Glossary
  - **Acceptance**: Navbar shows "Modules" dropdown, "Capstone", "Glossary" links in dev server

- [ ] T004 [US1] Configure footer in `docusaurus.config.js` with constitution link, GitHub repo link, license info
  - **Acceptance**: Footer displays all three links when viewing any page in dev server

- [ ] T005 [P] [US1] Create `src/css/custom.css` with educational theme (primary color #2E86AB, code block styling, mobile adjustments)
  - **Acceptance**: Dev server shows blue primary color on navbar and links

- [ ] T006 [P] [US1] Create `.github/workflows/deploy.yml` for GitHub Actions deployment to GitHub Pages
  - **Acceptance**: File exists with correct workflow syntax (checkout, setup-node, npm ci, npm run build, deploy to gh-pages)

- [ ] T007 [US1] Create initial `sidebars.js` with placeholder structure (index, quarter-overview, 4 module categories, capstone category, glossary)
  - **Acceptance**: Sidebar shows all 7 top-level entries (homepage, overview, 4 modules, capstone, glossary) in dev server

- [ ] T008 [P] [US1] Create `.gitignore` with node_modules, build, .docusaurus entries
  - **Acceptance**: Git status shows node_modules and build directories are ignored

- [ ] T009 [US1] Create root `README.md` with project overview, local development instructions (npm install, npm start, npm run build)
  - **Acceptance**: README includes all three npm commands and project description

- [ ] T010 [US1] Run `npm run build` locally and verify build succeeds without errors
  - **Acceptance**: Build completes, `build/` directory created with index.html

- [ ] T011 [US1] Commit Phase 1 changes and push to main branch to trigger GitHub Actions deployment
  - **Acceptance**: GitHub Actions workflow runs successfully, site visible at GitHub Pages URL

**Phase 1 Checkpoint**: ✅ Foundation complete - empty Docusaurus site deploys to GitHub Pages

---

## Phase 2: Foundational Content (User Story 1 - P1 MVP)

**Purpose**: Create complete navigation structure with all index pages (Quarter Overview, module indexes, capstone index, glossary)

**Checkpoint**: Students can navigate entire site structure using sidebar

### Homepage & Overview

- [ ] T012 [US1] Create `docs/index.mdx` with homepage welcome message, learning path visualization, quick links to Quarter Overview and Module 1
  - **Acceptance**: Page displays welcome header, lists 4 modules + capstone, includes "Get Started" button linking to quarter-overview

- [ ] T013 [US1] Create `docs/quarter-overview.mdx` with learning objectives section listing 5-7 bullet points
  - **Acceptance**: Page includes "Learning Objectives" heading with at least 5 curriculum-level goals

- [ ] T014 [US1] Add "Prerequisites & Preparation" section to `docs/quarter-overview.mdx` listing Python, Linux, CLI requirements
  - **Acceptance**: Section addresses Edge Case 4 (different student backgrounds) with prerequisite list and catch-up resources

- [ ] T015 [US1] Add module summaries to `docs/quarter-overview.mdx` with 2-3 sentence descriptions for each of 4 modules
  - **Acceptance**: Each module has a summary paragraph explaining key topics

- [ ] T016 [US1] Add expected timeline section to `docs/quarter-overview.mdx` (quarter/semester pacing, ~3 weeks per module, 2 weeks capstone)
  - **Acceptance**: Timeline section explains pacing and total duration

- [ ] T017 [US1] Add capstone project teaser to `docs/quarter-overview.mdx` linking to `/capstone/` page
  - **Acceptance**: Teaser paragraph explains integration project with link to capstone index

### Module 1 Index

- [ ] T018 [P] [US1] Create directory `docs/modules/module-01-robotic-nervous-system/`
  - **Acceptance**: Directory exists in repository

- [ ] T019 [US1] Create `docs/modules/module-01-robotic-nervous-system/index.mdx` with module title "Module 1: Robotic Nervous System"
  - **Acceptance**: Page displays h1 heading with module title

- [ ] T020 [US1] Add prerequisites section to Module 1 index (Python basics, command line familiarity)
  - **Acceptance**: "Prerequisites" section lists 2-3 requirements

- [ ] T021 [US1] Add learning objectives to Module 1 index (understand ROS 2 concepts, implement pub-sub, use services/actions)
  - **Acceptance**: "Learning Objectives" section lists 3-5 bullet points

- [ ] T022 [US1] Add lesson list to Module 1 index with placeholders for 3 lessons + 1 exercise
  - **Acceptance**: Bulleted list shows "Lesson 1: ROS 2 Basics", "Lesson 2: Publishers & Subscribers", "Lesson 3: Services & Actions", "Exercise: ROS 2 Communication"

- [ ] T023 [US1] Add exercise preview to Module 1 index describing hands-on ROS 2 workspace setup
  - **Acceptance**: Paragraph previews exercise outcome (students will set up workspace and run pub-sub example)

### Module 2 Index

- [ ] T024 [P] [US1] Create directory `docs/modules/module-02-digital-twin/`
  - **Acceptance**: Directory exists in repository

- [ ] T025 [US1] Create `docs/modules/module-02-digital-twin/index.mdx` with module title "Module 2: Digital Twin"
  - **Acceptance**: Page displays h1 heading with module title

- [ ] T026 [US1] Add prerequisites section to Module 2 index (Module 1 completion, ROS 2 basics)
  - **Acceptance**: "Prerequisites" section references Module 1

- [ ] T027 [US1] Add learning objectives to Module 2 index (understand simulation, use Gazebo, create URDF models)
  - **Acceptance**: "Learning Objectives" section lists 3-5 objectives

- [ ] T028 [US1] Add lesson list to Module 2 index with placeholders for 3 lessons + 1 exercise
  - **Acceptance**: Bulleted list shows "Lesson 1: Simulation Fundamentals", "Lesson 2: Gazebo Basics", "Lesson 3: URDF Robot Models", "Exercise: Humanoid Simulation"

- [ ] T029 [US1] Add exercise preview to Module 2 index describing Gazebo humanoid robot simulation
  - **Acceptance**: Paragraph previews launching humanoid robot in Gazebo environment

### Module 3 Index

- [ ] T030 [P] [US1] Create directory `docs/modules/module-03-ai-robot-brain/`
  - **Acceptance**: Directory exists in repository

- [ ] T031 [US1] Create `docs/modules/module-03-ai-robot-brain/index.mdx` with module title "Module 3: AI-Robot Brain"
  - **Acceptance**: Page displays h1 heading with module title

- [ ] T032 [US1] Add prerequisites section to Module 3 index (Modules 1-2, Python, basic AI concepts)
  - **Acceptance**: "Prerequisites" section references Modules 1-2

- [ ] T033 [US1] Add learning objectives to Module 3 index (understand planning, implement RL, use behavior trees)
  - **Acceptance**: "Learning Objectives" section lists 3-5 objectives

- [ ] T034 [US1] Add lesson list to Module 3 index with placeholders for 3 lessons + 1 exercise
  - **Acceptance**: Bulleted list shows "Lesson 1: Planning Algorithms", "Lesson 2: Reinforcement Learning", "Lesson 3: Behavior Trees", "Exercise: Navigation Planning"

- [ ] T035 [US1] Add exercise preview to Module 3 index describing navigation planner implementation
  - **Acceptance**: Paragraph previews using Nav2 stack for humanoid navigation

### Module 4 Index

- [ ] T036 [P] [US1] Create directory `docs/modules/module-04-vision-language-action/`
  - **Acceptance**: Directory exists in repository

- [ ] T037 [US1] Create `docs/modules/module-04-vision-language-action/index.mdx` with module title "Module 4: Vision-Language-Action"
  - **Acceptance**: Page displays h1 heading with module title

- [ ] T038 [US1] Add prerequisites section to Module 4 index (Modules 1-3, computer vision basics)
  - **Acceptance**: "Prerequisites" section references Modules 1-3

- [ ] T039 [US1] Add learning objectives to Module 4 index (understand VLA, implement vision pipeline, manipulation)
  - **Acceptance**: "Learning Objectives" section lists 3-5 objectives

- [ ] T040 [US1] Add lesson list to Module 4 index with placeholders for 3 lessons + 1 exercise
  - **Acceptance**: Bulleted list shows "Lesson 1: Computer Vision", "Lesson 2: VLA Models", "Lesson 3: Manipulation Tasks", "Exercise: Object Detection"

- [ ] T041 [US1] Add exercise preview to Module 4 index describing object detection with humanoid camera
  - **Acceptance**: Paragraph previews running YOLO on simulated camera feed

### Capstone & Glossary

- [ ] T042 [P] [US1] Create directory `docs/capstone/`
  - **Acceptance**: Directory exists in repository

- [ ] T043 [US1] Create `docs/capstone/index.mdx` with capstone overview (integration project requiring all 4 modules)
  - **Acceptance**: Page displays h1 "Capstone Project" and paragraph explaining integration requirement

- [ ] T044 [US1] Add capstone teaser to `docs/capstone/index.mdx` (example scenario: "Robot, pick up red cube")
  - **Acceptance**: Teaser describes end-to-end scenario with vision, planning, ROS 2 execution

- [ ] T045 [US1] Add links in capstone index to detailed pages (project-description, requirements, rubric - to be created later)
  - **Acceptance**: Three links present (may be placeholders initially)

- [ ] T046 [P] [US1] Create `docs/glossary.mdx` with title "Glossary" and alphabetical structure (A-Z headings)
  - **Acceptance**: Page displays "Glossary" h1 and alphabet section headers

- [ ] T047 [US1] Add initial glossary terms to `docs/glossary.mdx` (ROS 2, Node, Topic, Gazebo, URDF, VLA, Embodied AI, Humanoid Robot)
  - **Acceptance**: At least 8 terms defined with 1-2 sentence explanations and deep link anchors (FR-012)

### Sidebar Update

- [ ] T048 [US1] Update `sidebars.js` with complete navigation paths for all created pages (index, quarter-overview, 4 module indexes with lesson placeholders, capstone index, glossary)
  - **Acceptance**: Sidebar shows complete hierarchy in dev server, all links navigate to created pages (SC-005)

- [ ] T049 [US1] Test all navigation links in dev server by clicking through sidebar (no 404 errors)
  - **Acceptance**: All sidebar links resolve to valid pages (SC-005 partial validation)

**Phase 2 Checkpoint**: ✅ User Story 1 (P1) Complete - Students can navigate from homepage to any module/capstone index using sidebar

---

## Phase 3: Module 1 Content (User Story 2 - P2 Partial)

**Purpose**: Create complete Module 1 content with lessons, code examples, exercise, and diagrams

**Checkpoint**: Module 1 demonstrates full content pattern for replication in Modules 2-4

### Module 1 Lesson 1

- [ ] T050 [US2] Create `docs/modules/module-01-robotic-nervous-system/lesson-01-ros2-basics.mdx` with title "ROS 2 Basics"
  - **Acceptance**: Page displays h1 "ROS 2 Basics"

- [ ] T051 [US2] Add introduction paragraph to Lesson 1 explaining ROS 2 purpose (middleware for robot communication)
  - **Acceptance**: Intro paragraph defines ROS 2 in educational tone (FR-013)

- [ ] T052 [US2] Add "Core Concepts" section to Lesson 1 explaining nodes, topics, messages with humanoid context
  - **Acceptance**: Section defines all 3 concepts with humanoid robot examples (IMU sensor node, joint controller node - FR-014)

- [ ] T053 [US2] Add "Humanoid Robot Sensor Data Flow" subsection to Lesson 1 describing IMU and joint encoder data flow
  - **Acceptance**: Subsection explains sensor → controller → actuator flow for humanoid robots (Principle V)

- [ ] T054 [US2] Create diagram `static/img/modules/module-01/ros2-graph-humanoid.png` showing ROS 2 graph (sensor_node → /imu_data topic → control_node)
  - **Acceptance**: PNG file exists, diagram shows nodes as boxes, topics as arrows with labels

- [ ] T055 [US2] Embed diagram in Lesson 1 with caption "ROS 2 graph for humanoid robot sensor integration"
  - **Acceptance**: Image displays in lesson with caption using `![caption](path)` syntax (FR-008)

- [ ] T056 [US2] Add "Next Steps" section to Lesson 1 linking to Lesson 2 (Publishers & Subscribers)
  - **Acceptance**: Link navigates to lesson-02-publishers-subscribers when clicked

### Module 1 Lesson 2

- [ ] T057 [US2] Create `docs/modules/module-01-robotic-nervous-system/lesson-02-publishers-subscribers.mdx` with title "Publishers & Subscribers"
  - **Acceptance**: Page displays h1 "Publishers & Subscribers"

- [ ] T058 [US2] Add introduction paragraph to Lesson 2 explaining pub-sub pattern (decoupled communication)
  - **Acceptance**: Intro explains pattern in 2-3 sentences with educational tone

- [ ] T059 [US2] Add "Publisher Pattern" section to Lesson 2 explaining how publishers send messages to topics
  - **Acceptance**: Section includes definition and use case (sensor data publishing)

- [ ] T060 [US2] Create code file `static/code/module-01/imu_publisher.py` with complete IMU data publisher example (Python, ROS 2, syntactically correct)
  - **Acceptance**: Python file exists, imports rclpy and sensor_msgs.msg.Imu, defines publisher node, publishes mock IMU data

- [ ] T061 [US2] Test `static/code/module-01/imu_publisher.py` syntax with `flake8` or `python -m py_compile`
  - **Acceptance**: No syntax errors reported

- [ ] T062 [US2] Embed code example in Lesson 2 with ` ```python ` block and inline comments explaining each section
  - **Acceptance**: Code displays with syntax highlighting, comments explain purpose (FR-005, SC-006)

- [ ] T063 [US2] Add explanatory paragraph after code explaining how publisher sends IMU data for humanoid balance control
  - **Acceptance**: Paragraph describes code purpose in humanoid robotics context (FR-014)

- [ ] T064 [US2] Add "Subscriber Pattern" section to Lesson 2 explaining how subscribers receive messages from topics
  - **Acceptance**: Section includes definition and use case (controller receiving commands)

- [ ] T065 [US2] Create code file `static/code/module-01/joint_subscriber.py` with complete joint command subscriber example (Python, ROS 2)
  - **Acceptance**: Python file exists, imports rclpy, defines subscriber node, subscribes to /joint_commands topic

- [ ] T066 [US2] Test `static/code/module-01/joint_subscriber.py` syntax with `flake8` or `python -m py_compile`
  - **Acceptance**: No syntax errors reported

- [ ] T067 [US2] Embed joint subscriber code in Lesson 2 with ` ```python ` block and inline comments
  - **Acceptance**: Code displays with syntax highlighting, comments explain callback function (FR-005)

- [ ] T068 [US2] Add explanatory paragraph after subscriber code describing use in humanoid robot control
  - **Acceptance**: Paragraph connects code to humanoid actuator control (Principle V)

- [ ] T069 [US2] Create diagram `static/img/modules/module-01/message-flow-imu.png` showing message flow from sensor_node through /imu_data topic to control_node with arrows
  - **Acceptance**: PNG file exists, diagram shows data flow with direction arrows

- [ ] T070 [US2] Embed message flow diagram in Lesson 2 with caption "IMU data flow in humanoid robot system"
  - **Acceptance**: Image displays with caption (FR-008)

- [ ] T071 [US2] Add "Try It Yourself" callout box in Lesson 2 with instructions to download code files and test locally
  - **Acceptance**: Callout includes links to `static/code/module-01/imu_publisher.py` and `joint_subscriber.py`

### Module 1 Lesson 3

- [ ] T072 [US2] Create `docs/modules/module-01-robotic-nervous-system/lesson-03-services-actions.mdx` with title "Services & Actions"
  - **Acceptance**: Page displays h1 "Services & Actions"

- [ ] T073 [US2] Add introduction paragraph to Lesson 3 differentiating services (request-response) from actions (long-running tasks)
  - **Acceptance**: Intro explains both patterns in educational tone

- [ ] T074 [US2] Add "Service Pattern" section to Lesson 3 explaining synchronous request-response communication
  - **Acceptance**: Section defines services with humanoid example (balance calibration service)

- [ ] T075 [US2] Create code file `static/code/module-01/balance_service_client.py` with service client example for balance calibration
  - **Acceptance**: Python file exists, imports rclpy, defines service client, sends balance calibration request

- [ ] T076 [US2] Test `static/code/module-01/balance_service_client.py` syntax
  - **Acceptance**: No syntax errors reported

- [ ] T077 [US2] Embed service client code in Lesson 3 with ` ```python ` block and comments
  - **Acceptance**: Code displays with syntax highlighting, explains request/response (FR-005, SC-006)

- [ ] T078 [US2] Add explanatory paragraph describing balance calibration use case for humanoid stability
  - **Acceptance**: Paragraph explains why services fit this use case (Principle V)

- [ ] T079 [US2] Add "Action Pattern" section to Lesson 3 explaining asynchronous long-running tasks with feedback
  - **Acceptance**: Section defines actions with humanoid example (walking action with progress feedback)

- [ ] T080 [US2] Add pseudocode or conceptual example for walking action (no full implementation required)
  - **Acceptance**: Code block shows action client structure with goal, feedback, result

- [ ] T081 [US2] Add "When to Use Each Pattern" comparison table in Lesson 3 (topics vs services vs actions)
  - **Acceptance**: Table lists use cases for each pattern (streaming data → topics, one-time request → services, long tasks → actions)

### Module 1 Exercise

- [ ] T082 [US3] Create `docs/modules/module-01-robotic-nervous-system/exercise-ros2-communication.mdx` with title "Exercise: ROS 2 Communication Setup"
  - **Acceptance**: Page displays h1 "Exercise: ROS 2 Communication Setup"

- [ ] T083 [US3] Add "Overview" section to Exercise describing goal (set up ROS 2 workspace, run pub-sub example)
  - **Acceptance**: Overview paragraph states goal and estimated time (1-2 hours)

- [ ] T084 [US3] Add "Prerequisites" section to Exercise listing ROS 2 Humble installation, Ubuntu/Linux environment
  - **Acceptance**: Prerequisites list includes ROS 2 Humble with link to official installation guide (versioned link - Edge Case 3)

- [ ] T085 [US3] Add "Step 1: Install ROS 2 Humble" to Exercise with installation commands or link to official docs
  - **Acceptance**: Step includes command `sudo apt install ros-humble-desktop` or link to ros.org/humble/Installation

- [ ] T086 [US3] Add "Step 2: Create ROS 2 Workspace" to Exercise with workspace creation commands
  - **Acceptance**: Step includes `mkdir -p ~/ros2_ws/src`, `cd ~/ros2_ws`, `colcon build`

- [ ] T087 [US3] Add "Step 3: Copy Code Examples" to Exercise with instructions to download imu_publisher.py and joint_subscriber.py
  - **Acceptance**: Step includes links to code files in `/static/code/module-01/` and cp commands

- [ ] T088 [US3] Add "Step 4: Source Workspace" to Exercise with sourcing commands
  - **Acceptance**: Step includes `source ~/ros2_ws/install/setup.bash`

- [ ] T089 [US3] Add "Step 5: Run Publisher Node" to Exercise with command to run imu_publisher.py
  - **Acceptance**: Step includes `ros2 run <package> imu_publisher` (placeholder package name)

- [ ] T090 [US3] Add "Step 6: Run Subscriber Node (in new terminal)" to Exercise
  - **Acceptance**: Step includes opening second terminal and running subscriber

- [ ] T091 [US3] Add "Step 7: Visualize with rqt_graph" to Exercise showing ROS 2 graph visualization
  - **Acceptance**: Step includes `rqt_graph` command and describes expected output

- [ ] T092 [US3] Add "Expected Outcome" section to Exercise describing successful pub-sub communication (subscriber receives IMU data)
  - **Acceptance**: Section states "You should see IMU data printed in subscriber terminal" (FR-007)

- [ ] T093 [US3] Add "Troubleshooting" section to Exercise with 3-5 common errors and solutions
  - **Acceptance**: Troubleshooting includes "sourcing workspace" error, "package not found" error, "no messages received" issue (FR-007)

### Sidebar & Validation

- [ ] T094 [US2] Update `sidebars.js` to include all Module 1 lesson and exercise pages under Module 1 category
  - **Acceptance**: Sidebar shows Module 1 expanded with 3 lessons + 1 exercise in dev server

- [ ] T095 [US2] Test Module 1 content by clicking through all lessons and exercise pages (no broken links, images display, code highlights)
  - **Acceptance**: All pages render correctly, diagrams visible, code syntax-highlighted (SC-006 partial)

- [ ] T096 [US2] Verify Module 1 meets success criteria: 3 lessons, 2+ code examples, 1 exercise, 2 diagrams
  - **Acceptance**: Module 1 has lesson-01, lesson-02, lesson-03, exercise, 2 PNG diagrams, 3 Python files (SC-002, SC-003, SC-008 partial)

**Phase 3 Checkpoint**: ✅ Module 1 Complete - Pattern established for Modules 2-4 replication

---

## Phase 4: Module 2 Content (User Story 2 & 3 - P2/P3 Partial)

**Purpose**: Create complete Module 2 (Digital Twin) content following Module 1 pattern

**Checkpoint**: Module 2 complete with Gazebo simulation content

### Module 2 Lesson 1

- [ ] T097 [US2] Create `docs/modules/module-02-digital-twin/lesson-01-simulation-fundamentals.mdx` with title "Simulation Fundamentals"
  - **Acceptance**: Page displays h1 "Simulation Fundamentals"

- [ ] T098 [US2] Add introduction paragraph to Lesson 1 explaining why simulate (safe testing, rapid iteration, no hardware needed)
  - **Acceptance**: Intro explains simulation benefits in educational tone (FR-013)

- [ ] T099 [US2] Add "Digital Twin Concept" section to Lesson 1 defining digital twin (virtual replica of physical robot)
  - **Acceptance**: Section defines digital twin with humanoid robot example (FR-014)

- [ ] T100 [US2] Add "Simulation vs Reality" subsection comparing simulated and real humanoid robots
  - **Acceptance**: Subsection lists differences (perfect sensors in sim, physics approximations) and when to use each

- [ ] T101 [US2] Create diagram `static/img/modules/module-02/digital-twin-architecture.png` showing physical robot ↔ digital twin ↔ developer
  - **Acceptance**: PNG file exists, diagram shows bidirectional arrows between robot, simulation, and developer

- [ ] T102 [US2] Embed digital twin diagram in Lesson 1 with caption
  - **Acceptance**: Image displays with caption (FR-008)

### Module 2 Lesson 2

- [ ] T103 [US2] Create `docs/modules/module-02-digital-twin/lesson-02-gazebo-basics.mdx` with title "Gazebo Basics"
  - **Acceptance**: Page displays h1 "Gazebo Basics"

- [ ] T104 [US2] Add introduction paragraph to Lesson 2 explaining Gazebo (robotics simulator with physics engine)
  - **Acceptance**: Intro defines Gazebo and mentions version (Gazebo 11 or Garden - Principle III)

- [ ] T105 [US2] Add "World Setup" section to Lesson 2 explaining Gazebo worlds (environments with ground plane, obstacles)
  - **Acceptance**: Section describes world concept with humanoid navigation example

- [ ] T106 [US2] Add "Robot Spawning" section to Lesson 2 explaining how to load robot models into simulation
  - **Acceptance**: Section describes spawn process and mentions URDF/SDF formats

- [ ] T107 [US2] Create code file `static/code/module-02/humanoid_launch.py` with Gazebo launch file for humanoid robot (Python, ROS 2 launch syntax)
  - **Acceptance**: Python launch file exists, includes Gazebo launch, robot spawn, syntactically correct

- [ ] T108 [US2] Test `static/code/module-02/humanoid_launch.py` syntax
  - **Acceptance**: No syntax errors reported

- [ ] T109 [US2] Embed launch file code in Lesson 2 with ` ```python ` block and comments explaining each section
  - **Acceptance**: Code displays with syntax highlighting, comments explain Gazebo launch and spawn (FR-005)

- [ ] T110 [US2] Add explanatory paragraph describing launch file use for humanoid simulation
  - **Acceptance**: Paragraph explains how to run launch file and what happens (Gazebo opens with robot)

- [ ] T111 [US2] Add "Physics Engine" subsection to Lesson 2 briefly explaining gravity, collision, friction in Gazebo
  - **Acceptance**: Subsection mentions physics parameters relevant to humanoid stability

### Module 2 Lesson 3

- [ ] T112 [US2] Create `docs/modules/module-02-digital-twin/lesson-03-urdf-robot-models.mdx` with title "URDF Robot Models"
  - **Acceptance**: Page displays h1 "URDF Robot Models"

- [ ] T113 [US2] Add introduction paragraph to Lesson 3 explaining URDF (Unified Robot Description Format for robot structure)
  - **Acceptance**: Intro defines URDF in educational tone

- [ ] T114 [US2] Add "Links and Joints" section to Lesson 3 explaining robot components (links are rigid bodies, joints connect links)
  - **Acceptance**: Section defines links and joints with humanoid torso example

- [ ] T115 [US2] Add "Sensors in URDF" section to Lesson 3 explaining how to define sensors (cameras, IMU, lidar)
  - **Acceptance**: Section describes sensor tags with humanoid IMU example

- [ ] T116 [US2] Create code file `static/code/module-02/humanoid_torso.urdf` with URDF snippet for humanoid torso with IMU sensor (XML)
  - **Acceptance**: XML file exists, includes `<link>` for torso, `<sensor>` for IMU, valid URDF syntax

- [ ] T117 [US2] Test `static/code/module-02/humanoid_torso.urdf` syntax with URDF validator (or visual inspection)
  - **Acceptance**: URDF is well-formed XML with correct tags

- [ ] T118 [US2] Embed URDF snippet in Lesson 3 with ` ```xml ` block and comments
  - **Acceptance**: Code displays with syntax highlighting, comments explain link and sensor (FR-005, SC-006)

- [ ] T119 [US2] Add explanatory paragraph describing URDF structure for humanoid robots
  - **Acceptance**: Paragraph explains how URDF defines humanoid physical structure (Principle V)

- [ ] T120 [US2] Create diagram `static/img/modules/module-02/urdf-tree-structure.png` showing URDF tree (base_link → torso → head, arms, legs)
  - **Acceptance**: PNG file exists, diagram shows tree structure with link names

- [ ] T121 [US2] Embed URDF tree diagram in Lesson 3 with caption
  - **Acceptance**: Image displays with caption (FR-008)

### Module 2 Exercise

- [ ] T122 [US3] Create `docs/modules/module-02-digital-twin/exercise-humanoid-simulation.mdx` with title "Exercise: Launch Humanoid Simulation"
  - **Acceptance**: Page displays h1 "Exercise: Launch Humanoid Simulation"

- [ ] T123 [US3] Add "Overview" section describing goal (launch humanoid robot in Gazebo, observe standing behavior)
  - **Acceptance**: Overview states goal and estimated time

- [ ] T124 [US3] Add "Prerequisites" section listing Gazebo installation, ROS 2, Module 1 completion
  - **Acceptance**: Prerequisites include Gazebo 11/Garden installation link (versioned)

- [ ] T125 [US3] Add "Step 1: Install Gazebo" with installation commands or link
  - **Acceptance**: Step includes `sudo apt install gazebo` or link to gazebosim.org/docs

- [ ] T126 [US3] Add "Step 2: Download Humanoid Robot Model" with instructions to obtain URDF (could be link to public model or placeholder)
  - **Acceptance**: Step provides download link or git clone command for humanoid URDF

- [ ] T127 [US3] Add "Step 3: Copy Launch File" with instructions to download humanoid_launch.py
  - **Acceptance**: Step includes link to `/static/code/module-02/humanoid_launch.py`

- [ ] T128 [US3] Add "Step 4: Launch Gazebo with Robot" with command to run launch file
  - **Acceptance**: Step includes `ros2 launch <package> humanoid_launch.py`

- [ ] T129 [US3] Add "Expected Outcome" section describing successful simulation (Gazebo window opens, humanoid robot standing upright)
  - **Acceptance**: Section states "Robot should be standing in simulated environment" (FR-007)

- [ ] T130 [US3] Add "Troubleshooting" section with 3-5 common errors (model loading failure, Gazebo crash, physics instability)
  - **Acceptance**: Troubleshooting includes solutions for common issues (FR-007)

### Sidebar & Validation

- [ ] T131 [US2] Update `sidebars.js` to include all Module 2 lesson and exercise pages
  - **Acceptance**: Sidebar shows Module 2 with 3 lessons + 1 exercise

- [ ] T132 [US2] Test Module 2 content by clicking through all pages
  - **Acceptance**: All pages render correctly, diagrams visible, code highlighted

- [ ] T133 [US2] Verify Module 2 meets criteria: 3 lessons, 2+ code examples (launch file, URDF), 1 exercise, 2 diagrams
  - **Acceptance**: Module 2 has all required content (SC-002, SC-003, SC-008 partial)

**Phase 4 Checkpoint**: ✅ Module 2 Complete - Gazebo simulation content ready

---

## Phase 5: Module 3 Content (User Story 2 & 3 - P2/P3 Partial)

**Purpose**: Create complete Module 3 (AI-Robot Brain) content with planning and learning examples

**Checkpoint**: Module 3 complete with AI planning content

### Module 3 Lesson 1

- [ ] T134 [US2] Create `docs/modules/module-03-ai-robot-brain/lesson-01-planning-algorithms.mdx` with title "Planning Algorithms"
  - **Acceptance**: Page displays h1 "Planning Algorithms"

- [ ] T135 [US2] Add introduction paragraph explaining planning (computing action sequences to achieve goals)
  - **Acceptance**: Intro defines planning in educational tone (FR-013)

- [ ] T136 [US2] Add "Path Planning" section describing A* and RRT algorithms for humanoid navigation
  - **Acceptance**: Section explains both algorithms with humanoid locomotion context (FR-014)

- [ ] T137 [US2] Add "Motion Planning" section describing trajectory optimization for humanoid walking
  - **Acceptance**: Section explains motion planning with bipedal balance constraints

- [ ] T138 [US2] Create diagram `static/img/modules/module-03/planning-pipeline.png` showing perception → planning → execution loop
  - **Acceptance**: PNG file exists, diagram shows feedback loop for planning

- [ ] T139 [US2] Embed planning pipeline diagram in Lesson 1 with caption
  - **Acceptance**: Image displays with caption (FR-008)

- [ ] T140 [US2] Create code file `static/code/module-03/simple_planner.py` with simple A* or RRT pseudocode for navigation (Python)
  - **Acceptance**: Python file exists, shows planning algorithm structure (may be pseudocode)

- [ ] T141 [US2] Embed planner code in Lesson 1 with ` ```python ` block and comments
  - **Acceptance**: Code displays with comments explaining algorithm steps (FR-005)

### Module 3 Lesson 2

- [ ] T142 [US2] Create `docs/modules/module-03-ai-robot-brain/lesson-02-reinforcement-learning.mdx` with title "Reinforcement Learning for Robotics"
  - **Acceptance**: Page displays h1 "Reinforcement Learning for Robotics"

- [ ] T143 [US2] Add introduction paragraph explaining RL (learning optimal policies through trial and error)
  - **Acceptance**: Intro defines RL in educational tone

- [ ] T144 [US2] Add "Policy Learning" section explaining policy networks for humanoid control
  - **Acceptance**: Section describes policy mapping (state → action) for balance or walking

- [ ] T145 [US2] Add "Sim-to-Real Transfer" section describing training in simulation and deploying to real robots
  - **Acceptance**: Section explains simulation advantages and transfer challenges

- [ ] T146 [US2] Create diagram `static/img/modules/module-03/rl-training-loop.png` showing environment ↔ agent ↔ reward loop
  - **Acceptance**: PNG file exists, diagram shows RL training cycle

- [ ] T147 [US2] Embed RL training loop diagram in Lesson 2 with caption
  - **Acceptance**: Image displays with caption (FR-008)

- [ ] T148 [US2] Create code file `static/code/module-03/balance_rl_agent.py` with simple RL agent for balance control (Python, pseudocode acceptable)
  - **Acceptance**: Python file exists, shows agent structure (state observation, action selection)

- [ ] T149 [US2] Embed RL agent code in Lesson 2 with ` ```python ` block and comments
  - **Acceptance**: Code displays with comments explaining agent components (FR-005, SC-006)

### Module 3 Lesson 3

- [ ] T150 [US2] Create `docs/modules/module-03-ai-robot-brain/lesson-03-behavior-trees.mdx` with title "Behavior Trees"
  - **Acceptance**: Page displays h1 "Behavior Trees"

- [ ] T151 [US2] Add introduction paragraph explaining behavior trees (hierarchical task decomposition for complex behaviors)
  - **Acceptance**: Intro defines behavior trees in educational tone

- [ ] T152 [US2] Add "Tree Structure" section describing nodes (sequences, selectors, actions) and execution flow
  - **Acceptance**: Section explains tree execution with humanoid "approach object" example

- [ ] T153 [US2] Add "Humanoid Task Planning" section describing behavior tree for manipulation task (detect → approach → grasp)
  - **Acceptance**: Section describes task decomposition for humanoid manipulation (Principle V)

- [ ] T154 [US2] Create diagram `static/img/modules/module-03/behavior-tree-visualization.png` showing tree structure with nodes and connections
  - **Acceptance**: PNG file exists, diagram shows hierarchical tree for "approach object" task

- [ ] T155 [US2] Embed behavior tree diagram in Lesson 3 with caption
  - **Acceptance**: Image displays with caption (FR-008)

- [ ] T156 [US2] Create code file `static/code/module-03/approach_object.xml` with behavior tree XML for "approach object" task
  - **Acceptance**: XML file exists, defines behavior tree with sequence, actions (detect, navigate, grasp)

- [ ] T157 [US2] Embed behavior tree XML in Lesson 3 with ` ```xml ` block and comments
  - **Acceptance**: Code displays with comments explaining tree structure (FR-005)

### Module 3 Exercise

- [ ] T158 [US3] Create `docs/modules/module-03-ai-robot-brain/exercise-navigation-planning.mdx` with title "Exercise: Navigation Planning"
  - **Acceptance**: Page displays h1 "Exercise: Navigation Planning"

- [ ] T159 [US3] Add "Overview" section describing goal (implement navigation planner for humanoid in Gazebo using Nav2 stack)
  - **Acceptance**: Overview states goal and estimated time

- [ ] T160 [US3] Add "Prerequisites" section listing Nav2 installation, Modules 1-2, Gazebo humanoid model
  - **Acceptance**: Prerequisites include Nav2 installation link

- [ ] T161 [US3] Add "Step 1: Install Nav2" with installation commands
  - **Acceptance**: Step includes `sudo apt install ros-humble-navigation2`

- [ ] T162 [US3] Add "Step 2: Configure Nav2 for Humanoid" with instructions to create nav2_params.yaml
  - **Acceptance**: Step provides sample parameters or link to Nav2 docs

- [ ] T163 [US3] Add "Step 3: Launch Gazebo with Humanoid and Nav2" with launch command
  - **Acceptance**: Step includes `ros2 launch <package> nav2_humanoid.launch.py`

- [ ] T164 [US3] Add "Step 4: Set Navigation Goal in RViz" with instructions to use 2D Nav Goal tool
  - **Acceptance**: Step describes clicking in RViz to set goal pose

- [ ] T165 [US3] Add "Expected Outcome" section describing successful navigation (robot plans path and navigates to goal avoiding obstacles)
  - **Acceptance**: Section states expected behavior (FR-007)

- [ ] T166 [US3] Add "Troubleshooting" section with 3-5 common errors (Nav2 initialization failure, no path found, collision)
  - **Acceptance**: Troubleshooting includes solutions (FR-007)

### Sidebar & Validation

- [ ] T167 [US2] Update `sidebars.js` to include all Module 3 lesson and exercise pages
  - **Acceptance**: Sidebar shows Module 3 with 3 lessons + 1 exercise

- [ ] T168 [US2] Test Module 3 content by clicking through all pages
  - **Acceptance**: All pages render correctly, diagrams visible, code highlighted

- [ ] T169 [US2] Verify Module 3 meets criteria: 3 lessons, 2+ code examples (planner, RL agent, BT XML), 1 exercise, 2 diagrams
  - **Acceptance**: Module 3 has all required content (SC-002, SC-003, SC-008 partial)

**Phase 5 Checkpoint**: ✅ Module 3 Complete - AI planning and learning content ready

---

## Phase 6: Module 4 Content (User Story 2 & 3 - P2/P3 Complete)

**Purpose**: Create complete Module 4 (Vision-Language-Action) content with VLA and manipulation examples

**Checkpoint**: Module 4 complete - all module content finished

### Module 4 Lesson 1

- [ ] T170 [US2] Create `docs/modules/module-04-vision-language-action/lesson-01-computer-vision.mdx` with title "Computer Vision for Robotics"
  - **Acceptance**: Page displays h1 "Computer Vision for Robotics"

- [ ] T171 [US2] Add introduction paragraph explaining computer vision (enabling robots to perceive visual world)
  - **Acceptance**: Intro defines computer vision in educational tone (FR-013)

- [ ] T172 [US2] Add "Object Detection" section describing YOLO, Faster R-CNN for humanoid vision
  - **Acceptance**: Section explains detection with humanoid camera example (FR-014)

- [ ] T173 [US2] Add "Pose Estimation" section describing 6DOF object pose for manipulation
  - **Acceptance**: Section explains pose estimation for humanoid grasping

- [ ] T174 [US2] Create diagram `static/img/modules/module-04/vision-pipeline.png` showing camera → detection → pose estimation → grasp planning
  - **Acceptance**: PNG file exists, diagram shows vision to manipulation pipeline

- [ ] T175 [US2] Embed vision pipeline diagram in Lesson 1 with caption
  - **Acceptance**: Image displays with caption (FR-008)

- [ ] T176 [US2] Create code file `static/code/module-04/object_detector.py` with OpenCV-based object detection example (Python)
  - **Acceptance**: Python file exists, imports OpenCV and ROS 2, subscribes to camera topic, runs YOLO or similar detector

- [ ] T177 [US2] Test `static/code/module-04/object_detector.py` syntax
  - **Acceptance**: No syntax errors reported

- [ ] T178 [US2] Embed object detector code in Lesson 1 with ` ```python ` block and comments
  - **Acceptance**: Code displays with comments explaining detection pipeline (FR-005, SC-006)

### Module 4 Lesson 2

- [ ] T179 [US2] Create `docs/modules/module-04-vision-language-action/lesson-02-vla-models.mdx` with title "Vision-Language-Action Models"
  - **Acceptance**: Page displays h1 "Vision-Language-Action Models"

- [ ] T180 [US2] Add introduction paragraph explaining VLA (models that map language commands + vision to robot actions)
  - **Acceptance**: Intro defines VLA in educational tone

- [ ] T181 [US2] Add "VLA Architecture" section describing transformer-based models (RT-1, Octo) for humanoid control
  - **Acceptance**: Section explains VLA architecture with example command ("pick up red cube")

- [ ] T182 [US2] Add "Language Command Parsing" section describing how VLA interprets natural language
  - **Acceptance**: Section explains tokenization and command understanding

- [ ] T183 [US2] Create diagram `static/img/modules/module-04/vla-architecture.png` showing language input + vision input → VLA model → action output
  - **Acceptance**: PNG file exists, diagram shows VLA input-output flow

- [ ] T184 [US2] Embed VLA architecture diagram in Lesson 2 with caption
  - **Acceptance**: Image displays with caption (FR-008)

- [ ] T185 [US2] Add "Humanoid VLA Applications" section describing VLA use for humanoid manipulation and navigation
  - **Acceptance**: Section gives 2-3 examples of VLA commands for humanoid tasks (Principle V)

### Module 4 Lesson 3

- [ ] T186 [US2] Create `docs/modules/module-04-vision-language-action/lesson-03-manipulation-tasks.mdx` with title "Manipulation Tasks"
  - **Acceptance**: Page displays h1 "Manipulation Tasks"

- [ ] T187 [US2] Add introduction paragraph explaining manipulation (grasping, pick-and-place for humanoid hands)
  - **Acceptance**: Intro defines manipulation in educational tone

- [ ] T188 [US2] Add "Grasping Fundamentals" section describing grasp planning and execution
  - **Acceptance**: Section explains grasp planning with humanoid hand/gripper

- [ ] T189 [US2] Add "MoveIt2 Integration" section describing motion planning for humanoid arms using MoveIt2
  - **Acceptance**: Section explains MoveIt2 use for arm trajectory planning

- [ ] T190 [US2] Create diagram `static/img/modules/module-04/manipulation-pipeline.png` showing detect → plan grasp → move arm → close gripper
  - **Acceptance**: PNG file exists, diagram shows manipulation sequence

- [ ] T191 [US2] Embed manipulation pipeline diagram in Lesson 3 with caption
  - **Acceptance**: Image displays with caption (FR-008)

- [ ] T192 [US2] Create code file `static/code/module-04/arm_planner.py` with MoveIt2 arm planning example (Python)
  - **Acceptance**: Python file exists, imports MoveIt2 Python API, plans arm trajectory to target pose

- [ ] T193 [US2] Test `static/code/module-04/arm_planner.py` syntax
  - **Acceptance**: No syntax errors reported

- [ ] T194 [US2] Embed arm planner code in Lesson 3 with ` ```python ` block and comments
  - **Acceptance**: Code displays with comments explaining MoveIt2 planning (FR-005, SC-006)

### Module 4 Exercise

- [ ] T195 [US3] Create `docs/modules/module-04-vision-language-action/exercise-object-detection.mdx` with title "Exercise: Object Detection with Humanoid Camera"
  - **Acceptance**: Page displays h1 "Exercise: Object Detection with Humanoid Camera"

- [ ] T196 [US3] Add "Overview" section describing goal (run object detection on simulated humanoid camera feed)
  - **Acceptance**: Overview states goal and estimated time

- [ ] T197 [US3] Add "Prerequisites" section listing OpenCV installation, ROS 2, Gazebo humanoid with camera, Modules 1-3
  - **Acceptance**: Prerequisites include OpenCV installation command

- [ ] T198 [US3] Add "Step 1: Install OpenCV and YOLO" with installation commands
  - **Acceptance**: Step includes `pip install opencv-python` and link to YOLO weights

- [ ] T199 [US3] Add "Step 2: Launch Gazebo with Humanoid and Camera" with launch command
  - **Acceptance**: Step includes launch file that spawns humanoid with camera sensor

- [ ] T200 [US3] Add "Step 3: Run Object Detector Node" with command to run object_detector.py
  - **Acceptance**: Step includes `ros2 run <package> object_detector`

- [ ] T201 [US3] Add "Step 4: Visualize Detections in RViz" with instructions to view bounding boxes
  - **Acceptance**: Step describes opening RViz and subscribing to detection topic

- [ ] T202 [US3] Add "Expected Outcome" section describing successful detection (bounding boxes displayed on detected objects)
  - **Acceptance**: Section states "Bounding boxes appear around objects in camera view" (FR-007)

- [ ] T203 [US3] Add "Troubleshooting" section with 3-5 common errors (camera feed not publishing, YOLO weights missing, low detection accuracy)
  - **Acceptance**: Troubleshooting includes solutions (FR-007)

### Sidebar & Validation

- [ ] T204 [US2] Update `sidebars.js` to include all Module 4 lesson and exercise pages
  - **Acceptance**: Sidebar shows Module 4 with 3 lessons + 1 exercise

- [ ] T205 [US2] Test Module 4 content by clicking through all pages
  - **Acceptance**: All pages render correctly, diagrams visible, code highlighted

- [ ] T206 [US2] Verify Module 4 meets criteria: 3 lessons, 2+ code examples (object detector, arm planner), 1 exercise, 4 diagrams (vision pipeline, VLA arch, manipulation pipeline + 1 more)
  - **Acceptance**: Module 4 has all required content (SC-002, SC-003, SC-008 partial)

**Phase 6 Checkpoint**: ✅ All Modules Complete - User Stories 2 & 3 (P2, P3) finished

---

## Phase 7: Capstone Project Content (User Story 4 - P4)

**Purpose**: Create complete capstone project specification with integration requirements and rubric

**Checkpoint**: Capstone project content complete - entire book structure finished

### Capstone Project Description

- [ ] T207 [US4] Create `docs/capstone/project-description.mdx` with title "Capstone Project: Integrated Humanoid Robot System"
  - **Acceptance**: Page displays h1 "Capstone Project: Integrated Humanoid Robot System"

- [ ] T208 [US4] Add "Project Goal" section describing integration objective (build simulated humanoid with vision, planning, ROS 2 control)
  - **Acceptance**: Section states goal requires all 4 modules (FR-003)

- [ ] T209 [US4] Add "Integration Requirements" section listing how each module contributes to capstone
  - **Acceptance**: Section explicitly lists Module 1 (ROS 2), Module 2 (Gazebo), Module 3 (planning), Module 4 (vision) requirements

- [ ] T210 [US4] Add "Example Scenario" section describing end-to-end task ("Robot, pick up the red cube")
  - **Acceptance**: Scenario describes language command → vision detection → grasp planning → ROS 2 execution flow

- [ ] T211 [US4] Add "Timeline" section with weekly milestones for quarter/semester project
  - **Acceptance**: Timeline lists Week 1-10 milestones (setup, integration, testing, documentation)

- [ ] T212 [US4] Add links to detailed pages (requirements, rubric) from project description
  - **Acceptance**: Links navigate to requirements.mdx and rubric.mdx

### Capstone Requirements

- [ ] T213 [US4] Create `docs/capstone/requirements.mdx` with title "Technical Requirements"
  - **Acceptance**: Page displays h1 "Technical Requirements"

- [ ] T214 [US4] Add "System Components" section listing required technical components (R1-R7 from plan)
  - **Acceptance**: Section lists ROS 2 workspace, Gazebo world, humanoid model, detection node, planning node, language parser, launch file

- [ ] T215 [US4] Add "Module 1 Requirements" subsection describing ROS 2 nodes for sensor/actuator communication
  - **Acceptance**: Subsection specifies topics, messages, nodes required from Module 1

- [ ] T216 [US4] Add "Module 2 Requirements" subsection describing Gazebo world with obstacles and humanoid model
  - **Acceptance**: Subsection specifies simulation environment and robot model

- [ ] T217 [US4] Add "Module 3 Requirements" subsection describing planning node for navigation or manipulation
  - **Acceptance**: Subsection specifies planner choice (Nav2 or MoveIt2)

- [ ] T218 [US4] Add "Module 4 Requirements" subsection describing vision pipeline and language command parser
  - **Acceptance**: Subsection specifies object detection and command parsing requirements

- [ ] T219 [US4] Add "Deliverables" section listing source code, documentation, demonstration video
  - **Acceptance**: Section lists 3 deliverables with expected formats

### Capstone Rubric

- [ ] T220 [US4] Create `docs/capstone/rubric.mdx` with title "Evaluation Rubric"
  - **Acceptance**: Page displays h1 "Evaluation Rubric"

- [ ] T221 [US4] Add "Code Quality (20%)" criterion describing clean, commented, ROS 2 convention-following code
  - **Acceptance**: Criterion includes grading scale (excellent/good/fair/poor)

- [ ] T222 [US4] Add "Integration (30%)" criterion describing all 4 modules represented and communicating correctly
  - **Acceptance**: Criterion emphasizes cross-module communication (highest weight)

- [ ] T223 [US4] Add "Documentation (20%)" criterion describing clear architecture diagram and setup instructions
  - **Acceptance**: Criterion specifies README and architecture diagram requirements

- [ ] T224 [US4] Add "Demonstration (20%)" criterion describing video showing successful task completion
  - **Acceptance**: Criterion specifies video format and content (robot completing scenario)

- [ ] T225 [US4] Add "Complexity (10%)" criterion describing appropriate project scope for quarter timeline
  - **Acceptance**: Criterion assesses scope realism

- [ ] T226 [US4] Add "Grading Scale" section with A/B/C/D/F thresholds (90+/80+/70+/60+/<60)
  - **Acceptance**: Section lists percentage ranges for letter grades

- [ ] T227 [US4] Verify rubric has at least 5 evaluation criteria
  - **Acceptance**: Rubric includes Code Quality, Integration, Documentation, Demonstration, Complexity = 5 criteria (SC-007)

### Capstone Diagrams

- [ ] T228 [P] [US4] Create diagram `static/img/capstone/system-architecture.png` showing complete system with all ROS 2 nodes, topics, and data flow
  - **Acceptance**: PNG file exists, diagram shows camera → detection → planner → controller → actuators with all topics labeled

- [ ] T229 [P] [US4] Create diagram `static/img/capstone/integration-example.png` showing visual of robot detecting object and planning grasp
  - **Acceptance**: PNG file exists, diagram shows robot in Gazebo with bounding box on object and planned trajectory

- [ ] T230 [US4] Embed system architecture diagram in project-description.mdx
  - **Acceptance**: Image displays with caption (FR-008)

- [ ] T231 [US4] Embed integration example diagram in requirements.mdx
  - **Acceptance**: Image displays with caption

### Sidebar & Validation

- [ ] T232 [US4] Update `sidebars.js` to include all capstone pages (index, project-description, requirements, rubric) under Capstone category
  - **Acceptance**: Sidebar shows Capstone expanded with 4 pages

- [ ] T233 [US4] Test capstone content by clicking through all pages
  - **Acceptance**: All pages render correctly, diagrams visible

- [ ] T234 [US4] Verify capstone meets criteria: project description with all 4 module integration, requirements page, rubric with 5+ criteria, 2 diagrams
  - **Acceptance**: Capstone has all required content (SC-007, FR-015)

**Phase 7 Checkpoint**: ✅ User Story 4 (P4) Complete - Entire book structure finished

---

## Phase 8: Polish & Cross-Cutting Concerns

**Purpose**: Content review, link checking, mobile responsiveness, README, final deployment validation

**Checkpoint**: Production-ready site live on GitHub Pages

### Content Quality Review

- [ ] T235 [P] Review all module index pages for educational tone and clarity
  - **Acceptance**: All 4 module index pages use undergraduate-friendly language (FR-013)

- [ ] T236 [P] Review all lesson pages for educational tone and humanoid robot context
  - **Acceptance**: All 12 lessons maintain educational tone and mention humanoid robots (FR-013, FR-014)

- [ ] T237 [P] Verify all code examples have explanatory text
  - **Acceptance**: Every code block has surrounding paragraph explaining purpose in humanoid context (SC-006)

- [ ] T238 [P] Verify all exercises have troubleshooting sections
  - **Acceptance**: All 4 exercises include "Troubleshooting" section with 3+ common errors (FR-007)

- [ ] T239 [P] Verify all diagrams have captions and alt text
  - **Acceptance**: All images use `![alt text](path)` syntax with descriptive alt text

- [ ] T240 [P] Review glossary for completeness (all technical terms from lessons are defined)
  - **Acceptance**: Glossary includes at least 20 terms referenced in lessons with definitions (FR-012)

### Link Validation

- [ ] T241 Manually click through all sidebar links starting from homepage
  - **Acceptance**: All sidebar links navigate to valid pages (no 404 errors - SC-005)

- [ ] T242 Verify all external links include version numbers (ROS 2 Humble, Gazebo 11, etc.)
  - **Acceptance**: All links to ROS 2, Gazebo, Isaac documentation specify versions (Edge Case 3)

- [ ] T243 Check all glossary anchor links from lesson pages
  - **Acceptance**: Clicking glossary links navigates to correct term definition

- [ ] T244 Verify all code file download links in exercises
  - **Acceptance**: All links to `/static/code/` files download correctly

### Mobile Responsiveness

- [ ] T245 Test site on Chrome DevTools mobile emulator (iPhone SE, Pixel 5)
  - **Acceptance**: Sidebar collapses to hamburger menu on screens <768px (SC-010, Edge Case 1)

- [ ] T246 Verify code blocks are horizontally scrollable on mobile
  - **Acceptance**: Long code lines scroll horizontally without breaking layout (SC-010)

- [ ] T247 Test all diagrams display correctly on mobile
  - **Acceptance**: Images scale to fit mobile screen width without overflow

### Build & Deployment

- [ ] T248 Run `npm run build` locally and verify no errors
  - **Acceptance**: Build completes successfully, no errors in console

- [ ] T249 Verify build output size is <100MB
  - **Acceptance**: `build/` directory total size is under 100MB

- [ ] T250 Check build output for broken links using link checker tool (optional)
  - **Acceptance**: Link checker reports no broken internal links

- [ ] T251 Commit all Phase 8 changes and push to main branch
  - **Acceptance**: Git push succeeds, GitHub Actions workflow triggers

- [ ] T252 Verify GitHub Actions deployment succeeds
  - **Acceptance**: GitHub Actions workflow shows green checkmark (SC-004)

- [ ] T253 Visit deployed site on GitHub Pages and test homepage loads
  - **Acceptance**: Site loads at GitHub Pages URL with HTTPS

- [ ] T254 Test navigation through all pages on deployed site
  - **Acceptance**: All pages accessible on live site (SC-005)

### Documentation

- [ ] T255 Update root `README.md` with link to deployed GitHub Pages site
  - **Acceptance**: README includes live site URL in prominent location

- [ ] T256 Add "Project Overview" section to README describing educational book purpose
  - **Acceptance**: Overview paragraph explains Physical AI & Humanoid Robotics curriculum

- [ ] T257 Verify README includes local development instructions (npm install, npm start, npm run build)
  - **Acceptance**: README has "Local Development" section with all commands

- [ ] T258 Add "Contributing" section to README (if planning community contributions)
  - **Acceptance**: Contributing section explains how to submit issues or PRs (optional)

**Phase 8 Checkpoint**: ✅ Production-ready site deployed to GitHub Pages

---

## Phase 9: Validation & Final Checks

**Purpose**: Complete validation checklist, verify all success criteria, create maintenance documentation

**Checkpoint**: Project complete and handoff-ready

### Success Criteria Validation

- [ ] T259 Count total MDX pages in `/docs/` directory
  - **Acceptance**: Total is 26 pages (SC-001, SC-009): 1 homepage + 1 overview + 4 module indexes + 12 lessons + 4 exercises + 3 capstone pages + 1 glossary

- [ ] T260 Count code examples per module (verify 2+ per module)
  - **Acceptance**: Module 1 has 3, Module 2 has 2, Module 3 has 3, Module 4 has 2 = 10 total (SC-002)

- [ ] T261 Count simulation exercises (verify 1 per module)
  - **Acceptance**: 4 exercises total (1 per module - SC-003)

- [ ] T262 Verify GitHub Actions deployment succeeded
  - **Acceptance**: Latest workflow run shows green checkmark (SC-004)

- [ ] T263 Verify no broken navigation links (manual test or link checker)
  - **Acceptance**: All sidebar and in-content links work (SC-005)

- [ ] T264 Verify 100% of code examples have explanatory text
  - **Acceptance**: Manual review confirms all code blocks have surrounding context paragraphs (SC-006)

- [ ] T265 Count capstone rubric criteria
  - **Acceptance**: Rubric has 5 criteria (Code Quality, Integration, Documentation, Demonstration, Complexity - SC-007)

- [ ] T266 Count technical diagrams across all modules
  - **Acceptance**: Total is 10+ diagrams (2 per module + 2 capstone = 10 minimum - SC-008)

- [ ] T267 Verify total page count is 20-30
  - **Acceptance**: 26 pages falls within 20-30 range (SC-009)

- [ ] T268 Verify mobile responsiveness on <768px screens
  - **Acceptance**: Sidebar collapses, code scrolls horizontally (SC-010)

### Functional Requirements Validation

- [ ] T269 Verify FR-001: Quarter Overview page exists with learning objectives, module structure, timeline, prerequisites
  - **Acceptance**: docs/quarter-overview.mdx includes all 4 sections

- [ ] T270 Verify FR-002: Book includes 4 main modules with intro, lessons, exercises
  - **Acceptance**: All 4 module directories exist with index + 3 lessons + 1 exercise each

- [ ] T271 Verify FR-003: Capstone project exists describing integration project
  - **Acceptance**: docs/capstone/ includes project-description, requirements, rubric pages

- [ ] T272 Verify FR-004: Each module contains Python/ROS 2 code examples for humanoid robot control
  - **Acceptance**: All modules have 2+ code examples with humanoid robot context

- [ ] T273 Verify FR-005: Code examples are syntax-highlighted with explanatory text
  - **Acceptance**: All code blocks use ` ```python ` syntax and have context paragraphs

- [ ] T274 Verify FR-006: Book includes 1+ simulation exercise per module for Gazebo
  - **Acceptance**: All 4 modules have exercise pages with Gazebo instructions

- [ ] T275 Verify FR-007: Each exercise describes expected outcomes and troubleshooting
  - **Acceptance**: All 4 exercises include "Expected Outcome" and "Troubleshooting" sections

- [ ] T276 Verify FR-008: Book includes diagrams for system architecture, data flow, control loops
  - **Acceptance**: 10+ diagrams exist covering these topics

- [ ] T277 Verify FR-009: Book organized with hierarchical sidebar navigation
  - **Acceptance**: Sidebar reflects Quarter Overview → Modules 1-4 → Lessons → Capstone

- [ ] T278 Verify FR-010: All pages authored in MDX format compatible with Docusaurus
  - **Acceptance**: All files in docs/ have .mdx extension and render correctly

- [ ] T279 Verify FR-011: GitHub Pages deployment configuration exists and works
  - **Acceptance**: .github/workflows/deploy.yml exists and latest deployment succeeded

- [ ] T280 Verify FR-012: Technical terms defined on first use or linked to glossary
  - **Acceptance**: Glossary exists with 20+ terms, lessons link to glossary

- [ ] T281 Verify FR-013: Content maintains educational tone for undergraduate/graduate students
  - **Acceptance**: Manual review confirms clear, jargon-minimizing language

- [ ] T282 Verify FR-014: All humanoid robot examples specify simulated/real environment
  - **Acceptance**: Examples mention "in Gazebo simulation" or "simulated environment"

- [ ] T283 Verify FR-015: Capstone rubric defines code, documentation, demonstration requirements
  - **Acceptance**: Rubric includes all 3 requirement types

### Constitution Principles Validation

- [ ] T284 Verify Principle I (Educational-First Design): All content serves learning objectives, no marketing
  - **Acceptance**: Manual review confirms educational focus throughout

- [ ] T285 Verify Principle II (Hands-On Learning Mandate): Each module has 2+ code examples and 1 exercise
  - **Acceptance**: All modules meet this requirement (validated by SC-002, SC-003)

- [ ] T286 Verify Principle III (Tool Integration Standard): All examples use ROS 2, Gazebo, Python
  - **Acceptance**: Code examples use specified tools, no custom or obscure tools

- [ ] T287 Verify Principle IV (Modular Content Architecture): 4 modules organized as Quarter → Modules → Capstone
  - **Acceptance**: Structure matches specification

- [ ] T288 Verify Principle V (Humanoid-Centric Focus): All examples use humanoid robot context
  - **Acceptance**: Manual review confirms humanoid context in all lessons

- [ ] T289 Verify Principle VI (Deployment-Ready Documentation): MDX files, Docusaurus compatibility, GitHub Pages deployment
  - **Acceptance**: Site builds and deploys successfully

### Maintenance Documentation

- [ ] T290 Create `specs/001-robotics-education-book/validation.md` with validation checklist results
  - **Acceptance**: File documents all SC-001 through SC-010 validation results

- [ ] T291 Add "Architecture Decisions" section to validation.md documenting why Docusaurus, manual diagrams, no live execution, MDX
  - **Acceptance**: Section explains rationale for 4 key decisions

- [ ] T292 Create `specs/001-robotics-education-book/maintenance-guide.md` with instructions for adding new lessons
  - **Acceptance**: Guide explains how to create new MDX files and update sidebar

- [ ] T293 Add "Updating Code Examples" section to maintenance-guide.md describing how to update for new ROS 2 versions
  - **Acceptance**: Section provides step-by-step instructions for code updates

- [ ] T294 Add "Refreshing External Links" section to maintenance-guide.md for when ROS 2/Gazebo docs move
  - **Acceptance**: Section explains how to find and update external links

- [ ] T295 Create `specs/001-robotics-education-book/backlog.md` with future enhancement ideas
  - **Acceptance**: Backlog includes 5+ ideas (video tutorials, interactive playgrounds, forum integration, instructor dashboard, student analytics)

- [ ] T296 Verify all validation documentation is complete
  - **Acceptance**: validation.md, maintenance-guide.md, backlog.md all exist and are populated

**Phase 9 Checkpoint**: ✅ Project Complete - All validation passed, documentation complete, ready for handoff

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational Content (Phase 2)**: Depends on Setup completion - creates navigation structure
- **Module 1 (Phase 3)**: Depends on Foundational - establishes content pattern
- **Modules 2-4 (Phases 4-6)**: Depend on Module 1 completion - follow established pattern
- **Capstone (Phase 7)**: Depends on all Modules complete - references module concepts
- **Polish (Phase 8)**: Depends on all content complete - final review and deployment
- **Validation (Phase 9)**: Depends on Polish complete - verify all criteria met

### Critical Path

```
Phase 1 (T001-T011) → Phase 2 (T012-T049) → Phase 3 (T050-T096) → Phase 4 (T097-T133) → Phase 5 (T134-T169) → Phase 6 (T170-T206) → Phase 7 (T207-T234) → Phase 8 (T235-T258) → Phase 9 (T259-T296)
```

### Parallel Opportunities

**Within Setup (Phase 1)**:
- T005 (CSS), T006 (GitHub Actions), T008 (gitignore) can run in parallel after T001-T004

**Within Foundational (Phase 2)**:
- T018, T024, T030, T036, T042, T046 (directory/file creation) can run in parallel
- T235-T240 (content review in Phase 8) can be done by different reviewers in parallel

**Within Module Content (Phases 3-6)**:
- Diagrams for different modules can be created in parallel
- Code examples can be written and tested in parallel before embedding in lessons

### Task Count Summary

- **Phase 1 (Setup)**: 11 tasks (T001-T011)
- **Phase 2 (Foundational)**: 38 tasks (T012-T049)
- **Phase 3 (Module 1)**: 47 tasks (T050-T096)
- **Phase 4 (Module 2)**: 37 tasks (T097-T133)
- **Phase 5 (Module 3)**: 36 tasks (T134-T169)
- **Phase 6 (Module 4)**: 37 tasks (T170-T206)
- **Phase 7 (Capstone)**: 28 tasks (T207-T234)
- **Phase 8 (Polish)**: 24 tasks (T235-T258)
- **Phase 9 (Validation)**: 38 tasks (T259-T296)

**Total**: 296 atomic tasks

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (T001-T011)
2. Complete Phase 2: Foundational Content (T012-T049)
3. **STOP and VALIDATE**: Test complete navigation structure
4. Deploy and review with stakeholders

### Incremental Delivery

1. Complete Setup + Foundational → Navigation MVP ready
2. Add Module 1 (Phase 3) → First complete module for pattern validation
3. Add Modules 2-4 (Phases 4-6) → All module content complete
4. Add Capstone (Phase 7) → Full book structure complete
5. Polish + Validate (Phases 8-9) → Production-ready site

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together (Phases 1-2)
2. Once Module 1 pattern established (Phase 3):
   - Developer A: Module 2 (Phase 4)
   - Developer B: Module 3 (Phase 5)
   - Developer C: Module 4 (Phase 6)
3. Team collaborates on Capstone (Phase 7)
4. Distribute Polish tasks (Phase 8) by reviewer
5. One person completes Validation (Phase 9)

---

## Notes

- Each task produces ONE verifiable output (file, section, validation result)
- [P] tasks can run in parallel (different files, no dependencies)
- File paths are absolute from repository root
- Commit after each phase or logical group of tasks
- All code examples must be syntax-checked before embedding
- All diagrams must be created externally (draw.io) and saved as PNG
- Mobile testing should use Chrome DevTools emulator or real devices
- External links must include version numbers (ROS 2 Humble, Gazebo 11)
