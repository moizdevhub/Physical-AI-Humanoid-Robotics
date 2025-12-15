import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

const sidebars: SidebarsConfig = {
  tutorialSidebar: [
    'intro',
    'quarter-overview',
    {
      type: 'category',
      label: 'Module 1: Robotic Nervous System',
      collapsed: false,
      items: [
        'modules/module-01-robotic-nervous-system/index',
        'modules/module-01-robotic-nervous-system/lesson-01-ros2-basics',
        'modules/module-01-robotic-nervous-system/lesson-02-publishers-subscribers',
        'modules/module-01-robotic-nervous-system/lesson-03-services-actions',
        'modules/module-01-robotic-nervous-system/exercise-ros2-workspace',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Digital Twin',
      collapsed: false,
      items: [
        'modules/module-02-digital-twin/index',
        'modules/module-02-digital-twin/lesson-01-simulation-fundamentals',
        'modules/module-02-digital-twin/lesson-02-gazebo-basics',
        'modules/module-02-digital-twin/lesson-03-urdf-models',
        'modules/module-02-digital-twin/exercise-humanoid-gazebo',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: AI-Robot Brain',
      collapsed: false,
      items: [
        'modules/module-03-ai-robot-brain/index',
        'modules/module-03-ai-robot-brain/lesson-01-planning-algorithms',
        'modules/module-03-ai-robot-brain/lesson-02-reinforcement-learning',
        'modules/module-03-ai-robot-brain/lesson-03-behavior-trees',
        'modules/module-03-ai-robot-brain/exercise-navigation-planning',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action',
      collapsed: false,
      items: [
        'modules/module-04-vision-language-action/index',
        'modules/module-04-vision-language-action/lesson-01-computer-vision',
        'modules/module-04-vision-language-action/lesson-02-vision-language-action',
        'modules/module-04-vision-language-action/lesson-03-manipulation-tasks',
        'modules/module-04-vision-language-action/exercise-object-detection-grasping',
      ],
    },
    {
      type: 'category',
      label: 'Capstone Project',
      collapsed: false,
      items: [
        'capstone/index',
      ],
    },
    'glossary',
  ],
};

export default sidebars;
