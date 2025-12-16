# Physical AI & Humanoid Robotics Education

**Bridging Digital Intelligence and Physical Embodiment**

An educational Docusaurus-based book covering Physical AI and Humanoid Robotics for students learning to apply AI knowledge to control humanoid robots in simulated and real-world environments.

## Overview

This interactive learning platform guides students through four comprehensive modules covering the fundamentals of physical AI and humanoid robotics:

- **Module 1: Robotic Nervous System** - ROS 2 fundamentals, communication patterns, and sensor integration
- **Module 2: Digital Twin** - Simulation with Gazebo and NVIDIA Isaac, URDF robot models
- **Module 3: AI-Robot Brain** - Planning algorithms, reinforcement learning, and behavior trees
- **Module 4: Vision-Language-Action** - Computer vision, VLA models, and manipulation tasks
- **Capstone Project** - Integration project combining all modules

## Features

- 📚 **Modular Curriculum**: Self-contained modules that can be studied sequentially or selectively
- 💻 **Hands-On Learning**: Executable code examples with Python and ROS 2
- 🤖 **Humanoid-Centric**: All examples focus on humanoid robot applications
- 🎯 **Practical Exercises**: Simulation exercises with Gazebo and NVIDIA Isaac Sim
- 📊 **Technical Diagrams**: System architecture and data flow visualizations
- 📱 **Mobile-Responsive**: Accessible on all devices

## Quick Start

### Local Development

1. **Install Dependencies**
   ```bash
   npm install
   ```

2. **Start Development Server**
   ```bash
   npm start
   ```
   This command starts a local development server and opens up a browser window. Most changes are reflected live without having to restart the server.

3. **Build**
   ```bash
   npm run build
   ```
   This command generates static content into the `build` directory and can be served using any static contents hosting service.

4. **Serve Locally**
   ```bash
   npm run serve
   ```
   Serves the built website locally for testing before deployment.

## Project Structure

```
physical-ai-robotics/
├── docs/                          # Educational content (MDX files)
│   ├── intro.mdx                 # Homepage
│   ├── quarter-overview.mdx      # Curriculum overview
│   ├── modules/                  # Four main modules
│   ├── capstone/                 # Integration project
│   └── glossary.mdx             # Technical terms
├── static/                        # Static assets
│   ├── img/                      # Diagrams and images
│   └── code/                     # Downloadable code examples
├── src/                          # React components and styling
├── .github/workflows/            # GitHub Actions deployment
├── docusaurus.config.ts          # Site configuration
└── sidebars.ts                   # Navigation structure
```

## Technology Stack

- **Platform**: Docusaurus 3.x (React-based static site generator)
- **Language**: TypeScript for configuration, Python 3.8+ for code examples
- **Styling**: Tailwind CSS
- **Deployment**: GitHub Pages via GitHub Actions
- **Code Highlighting**: Prism
- **Educational Tools**: ROS 2 (Humble), Gazebo, NVIDIA Isaac Sim

## Constitution & Governance

This project follows strict educational principles defined in the [project constitution](https://github.com/Admin/AI-Native/blob/main/.specify/memory/constitution.md):

1. **Educational-First Design** - Focus on embodied intelligence learning
2. **Hands-On Learning Mandate** - Verifiable code examples and exercises
3. **Tool Integration Standard** - Industry-standard tools (ROS 2, Gazebo, Python)
4. **Modular Content Architecture** - Independent, self-contained modules
5. **Humanoid-Centric Focus** - All examples use humanoid robot context
6. **Deployment-Ready Documentation** - Docusaurus-compatible, auto-deployed

## Contributing

Contributions are welcome! Please ensure all content aligns with the educational mission and follows the constitution principles.

### Content Guidelines

- Maintain educational tone suitable for undergraduate/graduate students
- Include practical code examples for every major concept
- Ensure all examples specify humanoid robot context
- Provide step-by-step instructions for exercises
- Include troubleshooting guidance

## License

MIT License - See LICENSE file for details

## Links

- **Live Site**: https://Admin.github.io/AI-Native/ (GitHub Pages deployment)
- **GitHub Repository**: https://github.com/Admin/AI-Native
- **Project Constitution**: https://github.com/Admin/AI-Native/blob/main/.specify/memory/constitution.md
- **ROS 2 Documentation**: https://docs.ros.org/en/humble/

## Support

For issues, questions, or suggestions:
- Open an issue on GitHub
- Refer to the project documentation
- Check the glossary for technical terms

---

Built with ❤️ using Docusaurus | Educational Content for Physical AI & Humanoid Robotics
