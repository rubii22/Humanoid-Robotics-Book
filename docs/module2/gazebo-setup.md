# Chapter 4: The Digital Twin (Gazebo & Unity) - Gazebo Simulation Setup

## Overview

Welcome to Module 2: The Digital Twin (Gazebo & Unity). In this module, you'll learn how to create realistic simulation environments for humanoid robots using Gazebo, the physics-based simulation engine that's widely used in robotics research and development. We'll cover the fundamentals of Gazebo simulation, URDF/SDF model integration, physics parameters, and sensor simulation for humanoid robots.

## Learning Objectives

By the end of this module, you will be able to:

- Install and configure Gazebo simulation environment for humanoid robotics
- Create and import URDF/SDF models for humanoid robots
- Configure physics parameters for realistic humanoid simulation
- Simulate various sensors (LiDAR, cameras, IMUs) for humanoid robots
- Create custom Gazebo worlds for humanoid robot testing
- Integrate Gazebo with ROS 2 for closed-loop control

## Introduction to Gazebo for Humanoid Robotics

Gazebo is a powerful, physics-based simulation environment that enables the development, testing, and validation of robotics applications. For humanoid robots, Gazebo provides:

- **Realistic Physics Simulation**: Accurate modeling of contact forces, friction, and dynamics essential for bipedal locomotion
- **Sensor Simulation**: Realistic simulation of cameras, LiDAR, IMUs, and force/torque sensors
- **Flexible World Creation**: Custom environments for testing humanoid navigation and interaction
- **ROS Integration**: Seamless integration with ROS 2 for closed-loop control

### Why Gazebo for Humanoid Robots?

Humanoid robots present unique challenges that Gazebo addresses effectively:

1. **Complex Dynamics**: Bipedal locomotion requires accurate simulation of contact forces and balance
2. **Sensor Fusion**: Multiple sensors must be simulated and integrated for perception
3. **Environmental Interaction**: Humanoid robots interact with complex environments requiring detailed physics

## Installing Gazebo for Humanoid Robotics

### Prerequisites

Before installing Gazebo, ensure you have:

- Ubuntu 22.04 LTS
- ROS 2 Humble Hawksbill installed
- At least 8GB RAM (16GB recommended)
- GPU with OpenGL 3.3+ support

### Installing Gazebo Garden

Gazebo Garden is the latest stable version with excellent support for humanoid robotics:

```bash
# Add the Gazebo repository
sudo apt update && sudo apt install wget
wget https://packages.osrfoundation.org/gazebo.gpg -O /tmp/gazebo.gpg
sudo cp /tmp/gazebo.gpg /usr/share/keyrings/
echo "deb [arch=amd64 signed-by=/usr/share/keyrings/gazebo.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo.list > /dev/null

# Install Gazebo Garden
sudo apt update
sudo apt install gz-garden

# Install ROS 2 Gazebo packages
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control ros-humble-gazebo-dev
```

### Verifying Installation

Test your installation by launching Gazebo:

```bash
gz sim
```

## Creating Humanoid Robot Models in Gazebo

### URDF to SDF Conversion

Gazebo primarily uses SDF (Simulation Description Format), but can import URDF models. For humanoid robots, you'll typically design in URDF and convert or import directly:

```xml
<!-- Example: Simple humanoid torso for Gazebo -->
<sdf version='1.7'>
  <model name='humanoid_torso'>
    <link name='torso'>
      <pose>0 0 0.5 0 0 0</pose>
      <inertial>
        <mass>10.0</mass>
        <inertia>
          <ixx>0.1</ixx>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyy>0.1</iyy>
          <iyz>0.0</iyz>
          <izz>0.1</izz>
        </inertia>
      </inertial>

      <collision name='collision'>
        <geometry>
          <box>
            <size>0.3 0.2 0.5</size>
          </box>
        </geometry>
      </collision>

      <visual name='visual'>
        <geometry>
          <box>
            <size>0.3 0.2 0.5</size>
          </box>
        </geometry>
        <material>
          <ambient>0.8 0.8 0.8 1</ambient>
          <diffuse>0.8 0.8 0.8 1</diffuse>
        </material>
      </visual>

      <sensor name='imu_sensor' type='imu'>
        <always_on>true</always_on>
        <update_rate>100</update_rate>
      </sensor>
    </link>
  </model>
</sdf>
```

### Physics Properties for Humanoid Simulation

Humanoid robots require careful tuning of physics properties for realistic simulation:

```xml
<!-- Physics configuration for humanoid simulation -->
<physics type='ode'>
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
  <real_time_update_rate>1000</real_time_update_rate>
  <gravity>0 0 -9.8</gravity>

  <!-- Contact parameters for stable humanoid contact -->
  <ode>
    <solver>
      <type>quick</type>
      <iters>100</iters>
      <sor>1.3</sor>
    </solver>
    <constraints>
      <cfm>0.000001</cfm>
      <erp>0.2</erp>
      <contact_max_correcting_vel>100</contact_max_correcting_vel>
      <contact_surface_layer>0.001</contact_surface_layer>
    </constraints>
  </ode>
</physics>
```

## Sensor Integration for Humanoid Robots

### IMU Sensor Configuration

IMUs are critical for humanoid balance and orientation:

```xml
<sensor name='imu_sensor' type='imu'>
  <always_on>true</always_on>
  <update_rate>100</update_rate>
  <topic>imu/data</topic>
  <visualize>true</visualize>
  <imu>
    <angular_velocity>
      <x>
        <noise type='gaussian'>
          <mean>0.0</mean>
          <stddev>2e-4</stddev>
        </noise>
      </x>
      <y>
        <noise type='gaussian'>
          <mean>0.0</mean>
          <stddev>2e-4</stddev>
        </noise>
      </y>
      <z>
        <noise type='gaussian'>
          <mean>0.0</mean>
          <stddev>2e-4</stddev>
        </noise>
      </z>
    </angular_velocity>
    <linear_acceleration>
      <x>
        <noise type='gaussian'>
          <mean>0.0</mean>
          <stddev>1.7e-2</stddev>
        </noise>
      </x>
      <y>
        <noise type='gaussian'>
          <mean>0.0</mean>
          <stddev>1.7e-2</stddev>
        </noise>
      </y>
      <z>
        <noise type='gaussian'>
          <mean>0.0</mean>
          <stddev>1.7e-2</stddev>
        </noise>
      </z>
    </linear_acceleration>
  </imu>
</sensor>
```

### Camera and Depth Sensor Setup

Vision sensors for humanoid perception:

```xml
<sensor name='camera_sensor' type='camera'>
  <always_on>true</always_on>
  <update_rate>30</update_rate>
  <camera name='head_camera'>
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>10.0</far>
    </clip>
  </camera>
  <plugin filename='gz-sim-camera-system' name='gz::sim::systems::Camera'/>
</sensor>

<sensor name='depth_camera' type='depth_camera'>
  <always_on>true</always_on>
  <update_rate>30</update_rate>
  <camera name='depth_head_camera'>
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
    </image>
    <clip>
      <near>0.1</near>
      <far>10.0</far>
    </clip>
  </camera>
</sensor>
```

## Creating Custom Gazebo Worlds

### World File Structure

Create a custom world for humanoid testing:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="humanoid_test_world">
    <!-- Physics configuration -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <gravity>0 0 -9.8</gravity>
    </physics>

    <!-- Include outdoor environment -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Create a simple flat ground for humanoid walking -->
    <model name="flat_ground">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>1.0</mu>
                <mu2>1.0</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.7 0.7 0.7 1</ambient>
            <diffuse>0.7 0.7 0.7 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Add obstacles for navigation testing -->
    <model name="obstacle_1">
      <pose>2 0 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.5 0.5 1.0</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.5 0.5 1.0</size>
            </box>
          </geometry>
          <material>
            <ambient>0.5 0.2 0.8 1</ambient>
            <diffuse>0.5 0.2 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Add a ramp for testing walking dynamics -->
    <model name="ramp">
      <pose>5 0 0 0 0.2 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>2.0 1.0 0.1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>2.0 1.0 0.1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.3 0.3 0.3 1</ambient>
            <diffuse>0.3 0.3 0.3 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

## ROS 2 Integration with Gazebo

### Launching Gazebo with ROS 2

Create a launch file to start Gazebo with ROS 2 integration:

```python
# File: ~/humanoid_ws/src/humanoid_simulation/launch/humanoid_gazebo.launch.py

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch configuration variables
    use_rviz = LaunchConfiguration('use_rviz')
    world = LaunchConfiguration('world')

    # Declare launch arguments
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RViz')

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value='humanoid_test_world.sdf',
        description='Choose one of the world files from `/humanoid_simulation/worlds`')

    # Start Gazebo server
    start_gazebo_server_cmd = ExecuteProcess(
        cmd=['gz', 'sim', '-r', PathJoinSubstitution([
            FindPackageShare('humanoid_simulation'),
            'worlds',
            world
        ])],
        output='screen')

    # Start Gazebo client
    start_gazebo_client_cmd = ExecuteProcess(
        cmd=['gz', 'sim', '-g'],
        output='screen',
        condition=IfCondition(use_rviz))

    # RViz node
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('humanoid_simulation'),
        'rviz',
        'urdf.rviz'
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(use_rviz))

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare launch options
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_world_cmd)

    # Add commands to launch
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(rviz_node)

    return ld
```

### Robot State Publisher for Gazebo

```python
# File: ~/humanoid_ws/src/humanoid_simulation/launch/robot_state_publisher.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch configuration
    model_path = LaunchConfiguration('model_path')

    # Declare arguments
    declare_model_path_cmd = DeclareLaunchArgument(
        'model_path',
        default_value='$(find-pkg-share humanoid_description)/urdf/humanoid.urdf',
        description='Path to robot URDF file')

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': open(model_path.perform({})).read()
        }],
        output='screen'
    )

    # Joint state publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(declare_model_path_cmd)
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher)

    return ld
```

## Advanced Gazebo Features for Humanoid Robots

### Contact Sensors for Foot Detection

For humanoid locomotion, contact sensors on feet are essential:

```xml
<sensor name='left_foot_contact' type='contact'>
  <always_on>true</always_on>
  <update_rate>1000</update_rate>
  <contact>
    <collision>left_foot_collision</collision>
  </contact>
  <topic>left_foot_contact</topic>
</sensor>

<sensor name='right_foot_contact' type='contact'>
  <always_on>true</always_on>
  <update_rate>1000</update_rate>
  <contact>
    <collision>right_foot_collision</collision>
  </contact>
  <topic>right_foot_contact</topic>
</sensor>
```

### Force/Torque Sensors

Force/torque sensors for precise control:

```xml>
<sensor name='left_ankle_ft' type='force_torque'>
  <always_on>true</always_on>
  <update_rate>100</update_rate>
  <force_torque>
    <frame>child</frame>
    <measure_direction>child_to_parent</measure_direction>
  </force_torque>
</sensor>
```

## Unity Integration Overview

While Gazebo provides excellent physics simulation, Unity offers high-fidelity visualization for humanoid robotics applications. Unity can be used alongside Gazebo for:

- High-quality rendering and visualization
- Virtual reality integration
- Advanced graphics and lighting
- Realistic environment modeling

The integration typically involves:
1. Using Gazebo for physics simulation
2. Streaming pose data to Unity for visualization
3. Using Unity for perception simulation (synthetic data generation)

## Best Practices for Humanoid Simulation

### Physics Tuning

1. **Step Size**: Use small time steps (0.001s) for stable humanoid simulation
2. **Real-time Factor**: Balance between simulation speed and accuracy
3. **Contact Parameters**: Tune ERP and CFM for stable contact without jitter
4. **Solver Iterations**: Increase for more stable solutions

### Model Optimization

1. **Collision Simplification**: Use simplified collision geometry for performance
2. **Visual vs Collision**: Separate detailed visual models from simple collision models
3. **Inertial Properties**: Accurate mass distribution is critical for humanoid dynamics

### Sensor Configuration

1. **Update Rates**: Match sensor update rates to control system requirements
2. **Noise Parameters**: Configure realistic sensor noise models
3. **Mounting**: Accurate sensor placement relative to robot coordinate frames

## Summary

In this chapter, we've covered:

1. Gazebo installation and configuration for humanoid robotics
2. Creating humanoid robot models with appropriate physics properties
3. Configuring essential sensors for humanoid robots (IMU, cameras, contact sensors)
4. Creating custom worlds for humanoid testing
5. Integrating Gazebo with ROS 2 for closed-loop control
6. Advanced features for humanoid simulation (contact sensors, force/torque sensors)
7. Unity integration overview for high-fidelity visualization

The next chapter will explore NVIDIA Isaac integration for advanced perception and control pipelines.

## Exercises

1. Create a complete humanoid robot model with 20+ joints in Gazebo
2. Configure realistic physics parameters for stable bipedal walking
3. Set up a complete sensor suite including IMU, cameras, and contact sensors
4. Create a launch file that starts Gazebo with your humanoid robot and ROS 2 interfaces
5. Design a challenging world with obstacles for humanoid navigation testing