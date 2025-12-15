# Physical AI & Humanoid Robotics Education Constitution
<!--
Sync Impact Report:
Version: 1.0.0 → 1.0.0 (Initial constitution creation)
Rationale: Initial version establishing project governance for Docusaurus-based educational content on Physical AI and Humanoid Robotics.

Modified Principles:
- NEW: All six core principles defined from scratch

Added Sections:
- Core Principles (6 principles)
- Technical Standards
- Content Standards
- Governance

Removed Sections:
- None (initial creation)

Templates Status:
- ✅ plan-template.md - Compatible (constitution check section will use these principles)
- ✅ spec-template.md - Compatible (user stories align with educational focus)
- ✅ tasks-template.md - Compatible (task structure supports modular content development)

Follow-up TODOs:
- None - all placeholders resolved
-->

## Core Principles

### I. Educational-First Design
**MUST** maintain an educational focus on embodied intelligence and bridge the gap between digital AI and physical robotics. All content, examples, and exercises MUST serve the learning objective of helping students understand and implement AI systems in physical robots. Content that drifts into unrelated topics or lacks educational value violates this principle.

**Rationale**: This project exists to teach students practical AI-robotics integration. Entertainment, marketing, or tangential content dilutes the educational mission and confuses learners about what matters.

### II. Hands-On Learning Mandate
**MUST** provide verifiable, executable code examples and simulation exercises for every major concept. All tutorials MUST include step-by-step instructions that students can follow in either simulated environments (Gazebo, NVIDIA Isaac) or real-world robotic platforms. Theoretical discussions without practical implementation are not permitted as standalone content.

**Rationale**: Students learn robotics by doing, not just reading. Code that cannot be verified or exercises that cannot be completed fail the educational mission and frustrate learners.

### III. Tool Integration Standard
**MUST** integrate industry-standard tools: ROS 2 for robotic middleware, Gazebo for simulation, NVIDIA Isaac for advanced simulation, and Python for scripting. All code examples MUST follow ROS 2 conventions and best practices. Tool selection MUST prioritize what students will encounter in academic and industrial robotics settings.

**Rationale**: Students need transferable skills. Teaching custom or obscure tools limits their ability to collaborate, find jobs, and contribute to the robotics community.

### IV. Modular Content Architecture
**MUST** organize content into independent, self-contained modules that can be studied sequentially or selectively. The structure follows: Quarter Overview → Module 1 (Robotic Nervous System) → Module 2 (Digital Twin) → Module 3 (AI-Robot Brain) → Module 4 (Vision-Language-Action) → Capstone Project. Each module MUST be completable without requiring future modules, though prerequisite modules may be required.

**Rationale**: Different students have different backgrounds and time constraints. Modular design allows instructors to adapt the curriculum and students to focus on areas relevant to their goals.

### V. Humanoid-Centric Focus
**MUST** use humanoid robots as the primary application domain for all examples, simulations, and projects. When discussing sensors, actuators, control systems, or AI models, the context MUST be humanoid robotics in either simulated or real environments. Non-humanoid examples may be included for comparison or foundation-building but cannot dominate the content.

**Rationale**: Humanoid robotics represents a unique convergence of mechanical design, perception, planning, and human-robot interaction that provides rich learning opportunities across all AI-robotics domains.

### VI. Deployment-Ready Documentation
**MUST** maintain Docusaurus-compatible MDX files with proper frontmatter, sidebar navigation, and cross-references. All pages MUST render correctly in the Docusaurus development environment. The documentation site MUST be deployable to GitHub Pages via GitHub Actions without manual intervention. Markdown files that break the Docusaurus build are not acceptable.

**Rationale**: Students and instructors need a reliable, accessible web interface. Broken builds, missing navigation, or deployment failures undermine the project's usability and professional credibility.

## Technical Standards

### Platform & Deployment
- **Static Site Generator**: Docusaurus (version tracked in package.json)
- **Content Format**: MDX (Markdown with JSX support) for all documentation pages
- **Hosting**: GitHub Pages with automated deployment via GitHub Actions
- **Version Control**: Git-based workflow with feature branches for content additions

### Code Examples & Simulation
- **Primary Language**: Python 3.8+ for all robot control code, simulation scripts, and AI model integration
- **Robotic Middleware**: ROS 2 (Humble or later) for all communication, service, and action examples
- **Simulation Platforms**: Gazebo (primary) and NVIDIA Isaac Sim (advanced scenarios)
- **Dependencies**: MUST be installable via pip/apt with clear instructions in module README or setup guide

### File Organization
- **Module Structure**: Each module has its own directory under `/docs/modules/` with intro, lessons, exercises, and resources
- **Assets**: Images, diagrams, and videos stored in `/static/img/modules/<module-name>/`
- **Code Samples**: Standalone Python/ROS2 scripts in `/code/<module-name>/` with README instructions
- **Sidebar Configuration**: `sidebars.js` MUST reflect the Quarter → Module → Lesson hierarchy

## Content Standards

### Tone & Accessibility
- **Educational Tone**: Clear, patient, jargon-minimizing language suitable for undergraduate/graduate students new to robotics
- **Definitions**: Technical terms MUST be defined on first use with links to glossary or external references
- **Examples**: All concepts MUST be illustrated with concrete humanoid robot examples (e.g., balance control, grasping, navigation)

### Completeness & Verification
- **Code Correctness**: All code examples MUST be syntactically correct and tested in at least one target environment (Gazebo or real robot)
- **Exercise Testing**: All exercises MUST include expected outcomes or reference solutions for instructors
- **Diagrams**: System architecture, data flow, and control loop diagrams MUST accompany complex explanations

### Capstone Project
- **Integration**: The capstone project MUST require students to integrate concepts from all four modules
- **Deliverables**: Clear rubric defining code deliverables, documentation, and demonstration requirements
- **Realistic Scope**: Project MUST be completable in a quarter/semester timeline with typical student resources

## Governance

### Constitution Authority
This constitution supersedes all ad-hoc decisions. When in doubt, refer to the principles above. Any proposed change that conflicts with a principle MUST either:
1. Be rejected, OR
2. Trigger a constitution amendment process (see below)

### Amendment Process
1. **Proposal**: Document the proposed change with rationale, impact analysis, and migration plan
2. **Review**: Review must confirm alignment with educational mission and deployment requirements
3. **Approval**: Approval required before implementation (for solo developers: explicit commit message referencing this amendment)
4. **Migration**: Update all affected templates, content modules, and documentation

### Compliance Verification
- All pull requests MUST verify that new content aligns with principles I-VI
- Content that violates Educational-First Design (I), Hands-On Learning Mandate (II), or Tool Integration Standard (III) MUST be rejected or revised
- Docusaurus build MUST pass before merge (Deployment-Ready Documentation, VI)
- Complexity or exceptions MUST be justified in PR description or plan.md Complexity Tracking section

### Runtime Guidance
For day-to-day development and content creation decisions, refer to:
- `CLAUDE.md` - AI assistant behavior and development workflow
- Module-specific `README.md` files for content guidelines
- This constitution for governance and principles

**Version**: 1.0.0 | **Ratified**: 2025-12-15 | **Last Amended**: 2025-12-15
