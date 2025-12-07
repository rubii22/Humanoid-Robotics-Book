# Physical AI & Humanoid Robotics

Welcome to the Physical AI & Humanoid Robotics book project! This repository contains the documentation and code examples for the book "Physical AI & Humanoid Robotics — From Digital Brain to Embodied Intelligence".

## Getting Started

### Prerequisites

- Node.js 18+ (for Docusaurus documentation)
- Python 3.10+ (for ROS 2 packages)
- Ubuntu 22.04 LTS (recommended for ROS 2 Humble)

### Running the Documentation

1. Install Node.js dependencies:
   ```bash
   npm install
   ```

2. Start the Docusaurus development server:
   ```bash
   npm start
   ```

   This will start the documentation site on `http://localhost:3000` where you can browse the modules.

### Building the Documentation

To build the static site for deployment:
```bash
npm run build
```

### ROS 2 Packages

The `code/ros2_packages/` directory contains example ROS 2 packages for humanoid robotics:

- `basic_nodes`: Basic publisher/subscriber examples
- `humanoid_control`: Humanoid robot control nodes

## Project Structure

- `docs/`: Documentation files in Markdown format
- `code/`: ROS 2 packages and simulation examples
- `assets/`: Diagrams and model files
- `docusaurus.config.js`: Docusaurus configuration
- `sidebar.js`: Navigation sidebar configuration

## Modules

1. **Module 1**: The Robotic Nervous System (ROS 2)
2. **Module 2**: The Digital Twin (Gazebo & Unity)
3. **Module 3**: The AI-Robot Brain (NVIDIA Isaac)
4. **Module 4**: Vision-Language-Action (VLA) Pipelines
5. **Capstone**: Voice-Driven Autonomous Humanoid

## Contributing

For development, make sure to follow the existing documentation style and test examples before submitting changes.