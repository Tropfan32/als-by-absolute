import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

const sidebars: SidebarsConfig = {
  docsSidebar: [
    {
      type: 'category',
      label: 'Getting Started',
      collapsed: false,
      items: [
        'getting-started/introduction',
        'getting-started/prerequisites',
        'getting-started/installation',
      ],
    },
    {
      type: 'category',
      label: 'Core Architecture',
      items: [
        'architecture/overview',
        'architecture/impact-detector',
        'architecture/adaptive-path-planner',
        'architecture/foul-prevention-box',
      ],
    },
    {
      type: 'category',
      label: 'Setup Guides',
      items: [
        'setup/pinpoint-calibration',
        'setup/opmode-integration',
        'setup/tuning-guide',
      ],
    },
    {
      type: 'category',
      label: 'API Reference',
      items: [
        'api/impact-detector',
        'api/adaptive-path-planner',
        'api/foul-prevention-box',
        'api/point',
      ],
    },
  ],
};

export default sidebars;
