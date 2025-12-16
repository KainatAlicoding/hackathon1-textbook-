import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';
import Heading from '@theme/Heading';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <div className={styles.heroContent}>
          <div className={styles.textSection}>
            <Heading as="h1" className="hero__title">
              The Future of Physical AI & Humanoid Robotics
            </Heading>
            <p className="hero__subtitle">Master ROS 2, Gazebo, Isaac Sim, and VLA Models with this AI-Native Textbook.</p>
            <div className={styles.buttons}>
              <Link
                className="button button--secondary button--lg"
                to="/docs/intro">
                Explore the Physical AI & Humanoid Robotics Textbook
              </Link>
            </div>
          </div>
          <div className={styles.imageSection}>
            {/* TODO: Replace 'robot_hero_placeholder.png' in the static/img folder with a high-quality 3D render of a humanoid robot. */}
            <img
              src="img/robot_hero_placeholder.png"
              alt="Humanoid Robot"
              className={styles.heroImage}
            />
          </div>
        </div>
      </div>
    </header>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Hello from ${siteConfig.title}`}
      description="Description will go into a meta tag in <head />">
      <HomepageHeader />
      <main>
        <HomepageFeatures />
      </main>
    </Layout>
  );
}
