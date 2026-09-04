import type {ReactNode} from 'react';
import {useEffect, useState} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';

import styles from './index.module.css';

const GITHUB = 'https://github.com/aqqusr/aacs';
const VISUALIZER = 'https://visualizer.pedropathing.com';

const PHRASES = [
  'Reactive collision recovery',
  'Dynamic Bézier replanning',
  'Zero-GC execution',
];

const DOC_CARDS = [
  {
    title: 'Getting started',
    body: 'Install AACS, wire Pinpoint, and run the first OpMode loop.',
    to: '/docs/getting-started/introduction',
  },
  {
    title: 'Architecture',
    body: 'How the detector, planner, and geofence stay decoupled at 100+ Hz.',
    to: '/docs/architecture/overview',
  },
  {
    title: 'Setup guides',
    body: 'Calibration, OpMode integration, and recovery tuning.',
    to: '/docs/setup/pinpoint-calibration',
  },
  {
    title: 'API reference',
    body: 'Constructors, thresholds, and method contracts for every module.',
    to: '/docs/api/impact-detector',
  },
];

function CyclingPhrase() {
  const [index, setIndex] = useState(0);
  const [visible, setVisible] = useState(true);

  useEffect(() => {
    const reduce = window.matchMedia('(prefers-reduced-motion: reduce)');
    if (reduce.matches) {
      return undefined;
    }
    const id = window.setInterval(() => {
      setVisible(false);
      window.setTimeout(() => {
        setIndex((i) => (i + 1) % PHRASES.length);
        setVisible(true);
      }, 220);
    }, 3200);
    return () => window.clearInterval(id);
  }, []);

  return (
    <Heading
      as="h2"
      className={clsx(styles.cycleHeading, !visible && styles.phraseHidden)}>
      {PHRASES[index]}
    </Heading>
  );
}

function CodeWindow() {
  return (
    <div className={styles.codeCard}>
      <div className={styles.codeBar}>
        <span className={styles.dot} aria-hidden="true" />
        <span className={clsx(styles.dot, styles.dotAmber)} aria-hidden="true" />
        <span className={clsx(styles.dot, styles.dotGreen)} aria-hidden="true" />
        <span className={styles.codeTab}>AutonomousLoop.java</span>
      </div>
      <pre className={styles.codeBody}>
        <code>
          <span className={styles.cmt}>{'// Initialize AACS Pipeline'}</span>
          {'\n'}
          <span className={styles.type}>ImpactDetector</span> detector{' '}
          <span className={styles.kw}>=</span> <span className={styles.kw}>new</span>{' '}
          <span className={styles.type}>ImpactDetector</span>
          {'();\n'}
          <span className={styles.type}>AdaptivePathPlanner</span> planner{' '}
          <span className={styles.kw}>=</span> <span className={styles.kw}>new</span>{' '}
          <span className={styles.type}>AdaptivePathPlanner</span>
          {'(follower, detector);\n\n'}
          <span className={styles.cmt}>{'// Standard 100+ Hz Loop Contract'}</span>
          {'\n'}
          {'detector.'}
          <span className={styles.fn}>update</span>
          {'(axMetersPerSec2, ayMetersPerSec2);\n'}
          {'planner.'}
          <span className={styles.fn}>update</span>
          {'();\n'}
          {'follower.'}
          <span className={styles.fn}>update</span>
          {'();'}
        </code>
      </pre>
    </div>
  );
}

function Hero() {
  return (
    <header className={styles.hero}>
      <div className={clsx('container', styles.heroInner)}>
        <Heading
          as="h1"
          className={styles.heroTitle}
          style={{
            fontFamily: 'Benzin, sans-serif',
            fontWeight: 700,
            letterSpacing: '0.06em',
            lineHeight: 0.9,
          }}>
          AACS Framework
        </Heading>
        <p className={styles.heroSubtitle}>
          Ultra-fast, zero-GC collision recovery — by Absolute robotics for
          Pedro Pathing.
        </p>
        <div className={styles.ctaRow}>
          <Link
            className={styles.ctaPrimary}
            to="/docs/getting-started/introduction">
            Get started
          </Link>
          <Link className={styles.ctaGhost} href={GITHUB}>
            View on GitHub
          </Link>
        </div>
      </div>
    </header>
  );
}

function Showcase() {
  return (
    <section className={styles.section} aria-labelledby="companion-heading">
      <div className={clsx('container', styles.sectionInner)}>
        <div className={styles.showcaseGrid}>
          <div className={styles.showcaseCopy}>
            <p className={styles.kicker}>Companion tool</p>
            <div id="companion-heading">
              <CyclingPhrase />
            </div>
            <p className={styles.showcaseBody}>
              AACS watches Pinpoint acceleration every cycle, latches a real
              impact, and injects a velocity-aware quadratic Bézier back to the
              live target without pausing Pedro&apos;s follower. Recovery paths
              use the same <code>Path</code> / <code>BezierCurve</code> types as{' '}
              <Link href={VISUALIZER}>Pedro Pathing Visualizer</Link>, so
              Visualizer-authored routines and AACS replans stay compatible.
            </p>
          </div>
          <CodeWindow />
        </div>
      </div>
    </section>
  );
}

function DocsIndex() {
  return (
    <section className={styles.section} aria-labelledby="docs-heading">
      <div className={clsx('container', styles.sectionInner)}>
        <p className={styles.kicker}>Documentation</p>
        <Heading as="h2" id="docs-heading" className={styles.sectionTitle}>
          Start from a category
        </Heading>
        <div className={styles.docGrid}>
          {DOC_CARDS.map((card) => (
            <Link key={card.to} className={styles.docCard} to={card.to}>
              <h3 className={styles.docTitle}>{card.title}</h3>
              <p className={styles.docBody}>{card.body}</p>
              <span className={styles.docCta}>Open docs</span>
            </Link>
          ))}
        </div>
      </div>
    </section>
  );
}

export default function Home(): ReactNode {
  return (
    <Layout description="Ultra-fast, zero-GC collision recovery - by Absolute robotics for Pedro Pathing">
      <div className={styles.page}>
        <Hero />
        <Showcase />
        <DocsIndex />
      </div>
    </Layout>
  );
}
