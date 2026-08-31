import type {ReactNode} from 'react';
import {useEffect, useRef, useState} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';

import styles from './index.module.css';

const GITHUB = 'https://github.com/Tropfan32/als-by-absolute';
const VISUALIZER = 'https://visualizer.pedropathing.com';

const PHRASES = [
  'Reactive collision recovery',
  'Dynamic Bézier replanning',
  'Zero-GC execution',
];

function CyclingPhrase() {
  const [index, setIndex] = useState(0);
  const [visible, setVisible] = useState(true);

  useEffect(() => {
    const id = window.setInterval(() => {
      setVisible(false);
      window.setTimeout(() => {
        setIndex((i) => (i + 1) % PHRASES.length);
        setVisible(true);
      }, 280);
    }, 2800);
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
        <span className={styles.dot} />
        <span className={clsx(styles.dot, styles.dotAmber)} />
        <span className={clsx(styles.dot, styles.dotGreen)} />
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

function LavaLamp() {
  const rootRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    const root = rootRef.current;
    if (!root) {
      return;
    }

    let raf = 0;
    const apply = () => {
      const max = Math.max(
        1,
        document.documentElement.scrollHeight - window.innerHeight,
      );
      const t = Math.min(1, Math.max(0, window.scrollY / max));
      root.style.setProperty('--t', t.toFixed(4));
    };
    const onScroll = () => {
      cancelAnimationFrame(raf);
      raf = window.requestAnimationFrame(apply);
    };

    apply();
    window.addEventListener('scroll', onScroll, {passive: true});
    window.addEventListener('resize', onScroll, {passive: true});
    return () => {
      cancelAnimationFrame(raf);
      window.removeEventListener('scroll', onScroll);
      window.removeEventListener('resize', onScroll);
    };
  }, []);

  return (
    <div className={styles.lava} ref={rootRef} aria-hidden="true">
      <div className={clsx(styles.lavaTrack, styles.lavaPurple)}>
        <span className={styles.lavaBlob} />
      </div>
      <div className={clsx(styles.lavaTrack, styles.lavaPurpleSplit)}>
        <span className={clsx(styles.lavaBlob, styles.lavaBlobSmall)} />
      </div>
      <div className={clsx(styles.lavaTrack, styles.lavaPurpleSplitB)}>
        <span className={clsx(styles.lavaBlob, styles.lavaBlobSmall)} />
      </div>
      <div className={clsx(styles.lavaTrack, styles.lavaBlue)}>
        <span className={clsx(styles.lavaBlob, styles.lavaBlobBlue)} />
      </div>
    </div>
  );
}

function Hero() {
  return (
    <header className={styles.hero}>
      <div className={clsx('container', styles.heroInner)}>
        <Heading as="h1" className={styles.heroTitle}>
          AACS <span className={styles.gradientText}>Framework</span>
        </Heading>
        <p className={styles.heroSubtitle}>
          Ultra-fast, zero-GC collision recovery - by Absolute robotics for
          Pedro Pathing
        </p>
        <div className={styles.ctaRow}>
          <Link
            className={styles.ctaPrimary}
            to="/docs/getting-started/introduction">
            {'Get Started ->'}
          </Link>
          <Link className={styles.ctaGlass} href={GITHUB}>
            View on GitHub
          </Link>
        </div>
      </div>
    </header>
  );
}

function Showcase() {
  return (
    <section className={styles.showcase}>
      <div className={clsx('container', styles.showcaseInner)}>
        <div className={styles.showcaseGrid}>
          <div className={styles.showcaseCopy}>
            <p className={styles.kicker}>COMPANION TOOL</p>
            <CyclingPhrase />
            <p className={styles.showcaseBody}>
              AACS watches Pinpoint acceleration every cycle, latches a real
              impact, and injects a velocity-aware quadratic Bézier back to the
              live target without pausing Pedro&apos;s follower. Recovery paths
              use the same <code>Path</code> / <code>BezierCurve</code> types as{' '}
              <Link href={VISUALIZER}>Pedro Pathing Visualizer</Link>, so
              Visualizer-authored routines and AACS replans stay fully
              compatible.
            </p>
          </div>
          <CodeWindow />
        </div>
      </div>
    </section>
  );
}

export default function Home(): ReactNode {
  return (
    <Layout description="Ultra-fast, zero-GC collision recovery - by Absolute robotics for Pedro Pathing">
      <div className={styles.page}>
        <LavaLamp />
        <Hero />
        <Showcase />
      </div>
    </Layout>
  );
}
