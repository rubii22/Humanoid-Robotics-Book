// @ts-check
// Note: type annotations allow type checking and IDEs autocompletion

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'From Digital Brain to Embodied Intelligence',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: 'https://humanoid-robotics-book-inky.vercel.app/',
  // Set the /<base> pathname under which your site is served
  // For GitHub pages deployment, it is often '/<org-name>/<repo-name>'
  baseUrl: '/',

  // GitHub pages deployment config.
  organizationName: 'Rubab Fatima', // Usually your GitHub org/user name.
  projectName: 'Humanoid-Robotics-Book', // Usually your repo name.

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

  // Even if you don't use internalization, you can use this field to set useful
  // metadata like html lang. For example, if your site is Chinese, you may want
  // to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: require.resolve('./sidebar.js'),
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/facebook/docusaurus/tree/main/packages/create-docusaurus/templates/shared/',
        },
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      // Replace with your project's social card
      image: 'img/docusaurus-social-card.jpg',
      navbar: {
        title: 'Physical AI & Humanoid Robotics',
        hideOnScroll: false, // Keep navbar visible (sticky)
        style: 'primary',
        items: [
          {
            type: 'doc',
            docId: 'intro',
            position: 'left',
            label: 'Docs',
          },
          {
            href: 'https://github.com/rubii22',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Documentation',
            items: [
              {
                label: 'Introduction',
                to: '/docs/intro',
              },

            ],
          },
          {
            title: 'Resources',
            items: [
              {
                label: 'GitHub',
                href: 'https://github.com/rubii22',
              },
            ],
          },
          {
            title: 'More',
            items: [
              {
                label: 'Contact',
                href: 'https://x.com/DevRubabfatima',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Book. Built by Rubab Fatima.`,
      },
      prism: {
        theme: require('prism-react-renderer').themes.github,
        darkTheme: require('prism-react-renderer').themes.dracula,
      },
      homepage: {
        hero: {
          title: 'Physical AI & Humanoid Robotics',
          subtitle: 'A Practical Guide for Students, Makers, and Developers',
          actions: [
            {
              type: 'primary',
              label: 'Start Reading',
              to: '/docs/intro',
            },
          ],
        },
        features: [
          {
            title: 'Learn Physical AI',
            description: 'Principles of AI applied to real-world physical systems.',
            icon: '🤖',
          },
          {
            title: 'Explore Humanoid Robotics',
            description: 'Deep dive into modern humanoid robot technologies.',
            icon: '🦾',
          },
          {
            title: 'Build ROS 2 Systems',
            description: 'Build and deploy real robotic applications with ROS 2.',
            icon: '🔧',
          },
        ],
      },
    }),
};

module.exports = config;