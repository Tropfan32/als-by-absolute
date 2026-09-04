// @ts-check
const {themes: prismThemes} = require('prism-react-renderer');

const isHostedRoot = Boolean(process.env.VERCEL || process.env.CF_PAGES);

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'aacs',
  tagline: 'Autonomous Adaptive Control System',
  favicon: 'img/favicon.ico',

  future: {
    v4: true,
  },

  url: isHostedRoot ? 'https://aacs.vercel.app' : 'https://aqqusr.github.io',
  baseUrl: isHostedRoot ? '/' : '/aacs/',

  organizationName: 'aqqusr',
  projectName: 'aacs',
  deploymentBranch: 'gh-pages',
  trailingSlash: false,

  onBrokenLinks: 'throw',

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  markdown: {
    mermaid: true,
  },

  themes: ['@docusaurus/theme-mermaid'],

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: './sidebars.ts',
          routeBasePath: 'docs',
          editUrl:
            'https://github.com/aqqusr/aacs/tree/master/docs-site/',
        },
        blog: false,
        theme: {
          customCss: './src/css/custom.css',
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      image: 'img/docusaurus-social-card.jpg',
      colorMode: {
        defaultMode: 'dark',
        disableSwitch: true,
        respectPrefersColorScheme: false,
      },
      mermaid: {
        theme: {light: 'neutral', dark: 'dark'},
      },
      navbar: {
        title: 'AACS',
        hideOnScroll: false,
        logo: {
          alt: 'AACS',
          src: 'img/aacs-mark.png',
          srcDark: 'img/aacs-mark.png',
          href: '/',
          target: '_self',
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'docsSidebar',
            position: 'left',
            label: 'Docs',
          },
          {
            href: 'https://github.com/aqqusr/aacs',
            label: 'GitHub',
            position: 'right',
          },
          {
            href: 'https://visualizer.pedropathing.com',
            label: 'Visualizer',
            position: 'right',
          },
          {
            href: 'https://pedropathing.com',
            label: 'Pedro Pathing',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Docs',
            items: [
              {label: 'Getting started', to: '/docs/getting-started/introduction'},
              {label: 'Architecture', to: '/docs/architecture/impact-detector'},
              {label: 'API reference', to: '/docs/api/impact-detector'},
            ],
          },
          {
            title: 'Project',
            items: [
              {
                label: 'GitHub',
                href: 'https://github.com/aqqusr/aacs',
              },
              {
                label: 'Pedro Pathing',
                href: 'https://pedropathing.com',
              },
            ],
          },
          {
            title: 'Contact',
            items: [
              {
                label: 'Instagram',
                href: 'https://www.instagram.com/ftc_sunrise',
              },
              {
                label: 'Email',
                href: 'mailto:kairatnariman0001@gmail.com',
              },
              {
                label: 'News',
                href: 'https://t.me/sunriseeeetg',
              },
            ],
          },
        ],
        copyright: `© ${new Date().getFullYear()} AACS. All rights reserved.`,
      },
      prism: {
        theme: prismThemes.github,
        darkTheme: prismThemes.dracula,
        additionalLanguages: ['java', 'groovy', 'bash'],
      },
    }),
};

module.exports = config;
