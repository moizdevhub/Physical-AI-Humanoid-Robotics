import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
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
        // Lessons will be added in Phase 3
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Digital Twin',
      collapsed: false,
      items: [
        'modules/module-02-digital-twin/index',
        // Lessons will be added in Phase 4
      ],
    },
    {
      type: 'category',
      label: 'Module 3: AI-Robot Brain',
      collapsed: false,
      items: [
        'modules/module-03-ai-robot-brain/index',
        // Lessons will be added in Phase 5
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action',
      collapsed: false,
      items: [
        'modules/module-04-vision-language-action/index',
        // Lessons will be added in Phase 6
      ],
    },
    {
      type: 'category',
      label: 'Capstone Project',
      collapsed: false,
      items: [
        'capstone/index',
        // Project pages will be added in Phase 7
      ],
    },
    'glossary',
  ],
};

export default sidebars;
