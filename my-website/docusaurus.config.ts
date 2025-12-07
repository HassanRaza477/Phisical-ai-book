import { themes as prismThemes } from 'prism-react-renderer';
import type { Config } from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';


const config: Config = {
  title: "Physical AI & Humanoid Robotics",
  tagline: "AI-Native Textbook for the Future of Robotics",
  favicon: 'img/favicon.ico',

  
  future: {
    v4: true, // Improve compatibility with the upcoming Docusaurus v4
  },
  url: 'https://HassanRaza477.github.io',
  baseUrl: '/Phisical-ai-book',
  organizationName: 'HassanRaza477', 
  projectName: 'Phisical-ai-book', 
  onBrokenLinks: 'throw',
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/HassanRaza477/Phisical-ai-book.git',
        },
        blog: {
          showReadingTime: true,
          feedOptions: {
            type: ['rss', 'atom'],
            xslt: true,
          },
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/HassanRaza477/Phisical-ai-book.git',
          // Useful options to enforce blogging best practices
          onInlineTags: 'warn',
          onInlineAuthors: 'warn',
          onUntruncatedBlogPosts: 'warn',
        },
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {


    navbar: {
      style: "dark",
      title: "🤖 Humanoid Robotics",
      items: [
        {
          label: "📚 Book",
          position: "left",
          to: "/docs/intro",
          className: "navbar-book-btn",
        },
        {
          href: "https://github.com/yourrepo",
          label: "GitHub",
          position: "right",
          className: "navbar-github-btn",
        },
        {
          label: "Login",
          to: "/login", // Create a login page or link
          position: "right",
          className: "navbar-login-btn",
        },
        {
          label: "Sign Up",
          to: "/signup", // Create a signup page or link
          position: "right",
          className: "navbar-signup-btn",
        },
      ],
    },
    footer: {
      style: 'dark',
      logo: {
        alt: 'Physical AI & Humanoid Robotics Logo',
        src: '/img/logo-footer.svg',
        href: '/',
        width: 160,
        height: 40,
      },
      links: [
        {
          title: 'Textbook',
          items: [
            { label: 'Introduction', to: '/docs/intro' },
            { label: 'Physical AI', to: '/docs/physical-ai/overview' },
            { label: 'Humanoid Robotics', to: '/docs/humanoids/overview' },
            { label: 'Simulation', to: '/docs/simulation/overview' },
            { label: 'VLA Models', to: '/docs/advanced-ai/vla-models' },
            { label: 'Capstone', to: '/docs/capstone/autonomous-humanoid' },
          ],
        },
        {
          title: 'Tools',
          items: [
            { label: 'ROS 2', to: '/docs/tools/ros2' },
            { label: 'Gazebo + Unity', to: '/docs/tools/gazebo-unity' },
            { label: 'NVIDIA Isaac', to: '/docs/tools/isaac-sim' },
            { label: 'Code Examples', href: 'https://github.com/panaversity/physical-ai' },
            { label: 'API Reference', to: '/docs/tools/api' },
          ],
        },
        {
          title: 'Community',
          items: [
            { label: 'Panaversity Portal', href: 'https://ai-native.panaversity.org' },
            { label: 'Discord Community', href: 'https://discord.gg/physical-ai' },
            { label: 'X (Twitter)', href: 'https://x.com/panaversity_ai' },
            { label: 'GitHub', href: 'https://github.com/panaversity/physical-ai' },
            { label: 'YouTube', href: 'https://youtube.com/@panaversity' },
          ],
        },
        {
          title: 'Hackathon',
          items: [
            { label: 'Hackathon Details', href: 'https://ai-native.panaversity.org/hackathon' },
            { label: 'Register Team', href: 'https://ai-native.panaversity.org/register' },
            { label: 'Project Guidelines', to: '/docs/hackathon/guidelines' },
            { label: 'Judging Criteria', to: '/docs/hackathon/judging' },
            { label: 'Mentors', to: '/docs/hackathon/mentors' },
          ],
        },
      ],
      copyright: `© ${new Date().getFullYear()} Physical AI & Humanoid Robotics — Panaversity. Built with Docusaurus and AI Agents.`,
    },



    stylesheets: [
      {
        href: 'https://fonts.googleapis.com/css2?family=Inter:wght@400;500;600;700;800&display=swap',
        type: 'text/css',
      },
    ],

    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
