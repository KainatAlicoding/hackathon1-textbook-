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
    title: 'Advanced Robotics Curriculum',
    description: (
      <>
        Comprehensive modules covering ROS 2, Gazebo physics simulation, NVIDIA Isaac Sim,
        and Vision-Language-Action (VLA) models for next-generation robotics.
      </>
    ),
  },
  {
    title: 'AI-Native Learning',
    description: (
      <>
        Integrated RAG Chatbot powered by Gemini provides personalized learning
        assistance and answers to complex robotics concepts in real-time.
      </>
    ),
  },
  {
    title: 'Hands-On Experience',
    description: (
      <>
        Practical exercises and projects that let you apply robotics concepts
        in simulated environments with Unity and Isaac Sim integration.
      </>
    ),
  },
];

function Feature({title, description}: FeatureItem) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center padding-horiz--md">
        <div className={styles.featureIcon}>
          <div className={styles.robotIcon}>🤖</div>
        </div>
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
