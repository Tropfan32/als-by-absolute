import type {ReactNode} from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

type FeatureItem = {
  title: string;
  description: ReactNode;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'Detect the hit',
    description: (
      <>
        <code>ImpactDetector</code> watches horizontal acceleration and latches
        only after a sustained G spike — vibration does not count.
      </>
    ),
  },
  {
    title: 'Replan without stopping',
    description: (
      <>
        <code>AdaptivePathPlanner</code> injects a velocity-aware quadratic
        Bézier from the live Pinpoint pose back to the current target.
      </>
    ),
  },
  {
    title: 'Stay legal',
    description: (
      <>
        <code>FoulPreventionBox</code> geofences Alliance / opponent zones so
        recovery curves cannot clip illegal AABBs or field walls.
      </>
    ),
  },
];

function Feature({title, description}: FeatureItem) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center padding-horiz--md">
        <Heading as="h3">{title}</Heading>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures(): ReactNode {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
