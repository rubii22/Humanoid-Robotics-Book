import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <div className={styles.heroContent}>
          <div className={styles.heroBackgroundShape}></div>
          <h1 className={clsx('hero__title', styles.mainTitle)}>
            {siteConfig.title}
          </h1>
          <p className={clsx('hero__subtitle', styles.mainSubtitle)}>
            {siteConfig.tagline}
          </p>
          <p className={clsx('hero__subtitle', styles.secondarySubtitle)}>
            A Practical Guide for Students, Makers, and Developers
          </p>
          <div className={styles.buttons}>
            <Link
              className={clsx('button button--primary button--lg', styles.startButton)}
              to="/docs/intro">
              Start Reading
            </Link>
            <Link
              className={clsx('button button--secondary button--lg', styles.githubButton)}
              href="https://github.com/rubii">
              GitHub
            </Link>
          </div>
        </div>
      </div>
    </header>
  );
}

function FeatureCard({ title, description, icon }) {
  return (
    <div className={clsx('col col--4', styles.featureCard)}>
      <div className={clsx(styles.cardContent, styles.cardGlass)}>
        <div className={styles.cardIcon}>
          {icon}
        </div>
        <h3 className={styles.cardTitle}>{title}</h3>
        <p className={styles.cardDescription}>{description}</p>
      </div>
    </div>
  );
}

function FeaturesSection() {
  const features = [
    {
      title: 'Robotics',
      description: 'Principles of physical AI systems',
      icon: '🤖'
    },
    {
      title: 'Computer Vision',
      description: 'Visual perception for autonomous systems',
      icon: '👁️'
    },
    {
      title: 'AI & Machine Learning',
      description: 'Intelligent decision making for robots',
      icon: '🧠'
    },
    {
      title: 'Motion Control',
      description: 'Advanced locomotion and manipulation',
      icon: '🦾'
    },
    {
      title: 'Hardware Integration',
      description: 'Sensors, actuators, and embedded systems',
      icon: '⚙️'
    },
    {
      title: 'ROS 2 Framework',
      description: 'Build and deploy real robotic applications',
      icon: '🔧'
    }
  ];

  return (
    <section className={styles.featuresSection}>
      <div className="container">
        <div className="row">
          {features.map((feature, index) => (
            <FeatureCard
              key={index}
              title={feature.title}
              description={feature.description}
              icon={feature.icon}
            />
          ))}
        </div>
      </div>
    </section>
  );
}

function BookOverviewSection() {
  return (
    <section className={styles.bookOverviewSection}>
      <div className="container">
        <div className="row">
          <div className="col col--6">
            <h2 className={styles.overviewTitle}>Complete Guide to Physical AI & Humanoid Robotics</h2>
            <p className={styles.overviewDescription}>
              This comprehensive book provides a practical approach to understanding and implementing
              humanoid robotics systems. From the fundamentals of ROS 2 to advanced AI integration,
              you'll learn to build sophisticated robotic systems that interact with the physical world.
            </p>
            <ul className={styles.overviewList}>
              <li>Master ROS 2 fundamentals for humanoid robot control</li>
              <li>Build digital twins with Gazebo and Unity simulation</li>
              <li>Integrate NVIDIA Isaac tools for perception and navigation</li>
              <li>Develop Vision-Language-Action (VLA) pipelines</li>
              <li>Deploy autonomous humanoid behaviors</li>
            </ul>
          </div>
          <div className="col col--6">
            <div className={clsx(styles.overviewCard, styles.cardGlass)}>
              <h3>What You'll Build</h3>
              <p>By the end of this book, you'll have created a voice-controlled autonomous humanoid robot
                capable of understanding and executing complex commands in real-world environments.</p>
            </div>
            <div className={clsx(styles.overviewCard, styles.cardGlass, styles.overviewCardSecondary)}>
              <h3>Target Audience</h3>
              <p>Ideal for AI/ML students, engineers new to embodied intelligence,
                and educators building Physical-AI courses.</p>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Welcome to ${siteConfig.title}`}
      description="A Practical Guide for Students, Makers, and Developers">
      <HomepageHeader />
      <main>
        <FeaturesSection />
        <BookOverviewSection />
      </main>
    </Layout>
  );
}