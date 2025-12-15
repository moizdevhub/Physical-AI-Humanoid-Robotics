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
      ],
    },
    {
      type: 'category',
      label: 'Module 3: AI-Robot Brain',
      collapsed: false,
      items: [
        'modules/module-03-ai-robot-brain/index',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action',
      collapsed: false,
      items: [
        'modules/module-04-vision-language-action/index',
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
