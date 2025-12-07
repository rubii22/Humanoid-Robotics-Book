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
              className={clsx('button button--secondary button--lg', styles.startButton)}
              to="/docs/intro">
              Start Reading
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
      <div className={styles.cardContent}>
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
      title: 'Learn Physical AI',
      description: 'Principles of physical AI systems',
      icon: '🧠'
    },
    {
      title: 'Explore Humanoid Robotics',
      description: 'Modern humanoid robot technologies',
      icon: '🤖'
    },
    {
      title: 'Build ROS 2 Systems',
      description: 'Build and deploy real ROS 2 applications',
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

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Welcome to ${siteConfig.title}`}
      description="A Practical Guide for Students, Makers, and Developers">
      <HomepageHeader />
      <main>
        <FeaturesSection />
      </main>
    </Layout>
  );
}