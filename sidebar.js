// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Introduction',
      items: ['intro'],
    },
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        'module1/ros2-intro',
        'module1/ros2-rclpy',
        'module1/logging-guide'
      ],
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      items: [
        'module2/gazebo-setup'
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac)',
      items: [
        'module3/isaac-setup'
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA) Pipelines',
      items: [
        'module4/vla-intro'
      ],
    },
    {
      type: 'category',
      label: 'Capstone: Autonomous Humanoid',
      items: [
        'capstone/capstone'
      ],
    },
    {
      type: 'category',
      label: 'Appendices',
      items: [
        'hardware-setup',
        'references'
      ],
    }
  ],
};

module.exports = sidebars;