# Chapter 1: The Robotic Nervous System (ROS 2) - Introduction

## Overview

Welcome to Module 1: The Robotic Nervous System (ROS 2). In this module, you'll learn the fundamentals of ROS 2 architecture specifically for humanoid robotics applications. We'll cover the core concepts of ROS 2, including nodes, topics, services, and actions, and apply them to humanoid robot control.

## Learning Objectives

By the end of this module, you will be able to:

- Understand the fundamental concepts of ROS 2 architecture
- Create and run basic ROS 2 nodes for humanoid robot control
- Implement publishers and subscribers for robot sensor data
- Work with ROS 2 services and actions for humanoid robot operations
- Create URDF models for humanoid robots
- Use launch files to start complex humanoid robot systems

## ROS 2 Architecture for Humanoid Robots

ROS 2 (Robot Operating System 2) provides a flexible framework for writing robot software. For humanoid robots, ROS 2 offers:

- **Modularity**: Different robot subsystems (e.g., walking, manipulation, perception) can be developed as independent nodes
- **Real-time capabilities**: With proper configuration, ROS 2 can meet the timing requirements for humanoid robot control
- **Distributed computing**: Different parts of the humanoid robot system can run on different computers
- **Rich ecosystem**: Extensive libraries for navigation, manipulation, and perception specifically designed for robotics

### Core Concepts

#### Nodes
A node is a process that performs computation. In a humanoid robot, you might have nodes for:
- Joint control
- Sensor processing
- Walking pattern generation
- Perception
- High-level planning

#### Topics
Topics enable asynchronous message passing between nodes. Common topics in humanoid robots include:
- `/joint_states`: Current joint positions, velocities, and efforts
- `/cmd_vel`: Velocity commands for base movement
- `/sensor_data`: Sensor readings from IMUs, cameras, etc.

#### Services
Services provide synchronous request/response communication. Examples:
- `/set_parameters`: Configure robot parameters
- `/get_state`: Request current robot state

#### Actions
Actions are used for long-running tasks with feedback. In humanoid robots:
- `/move_base`: Navigate to a goal location
- `/follow_joint_trajectory`: Execute a joint trajectory with feedback

## Setting Up Your Development Environment

### Prerequisites

Before starting, ensure you have:

- Ubuntu 22.04 LTS installed
- At least 16GB RAM (64GB recommended for simulation)
- Python 3.10 or higher

### Installing ROS 2 Humble Hawksbill

ROS 2 Humble Hawksbill is the recommended version for this book as it has long-term support and good compatibility with humanoid robotics packages.

```bash
# Add the ROS 2 apt repository
sudo apt update && sudo apt install -y software-properties-common
sudo add-apt-repository universe

# Add the ROS 2 GPG key and repository
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 development packages
sudo apt update
sudo apt install -y ros-humble-desktop
sudo apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-build-essential

# Initialize rosdep
sudo rosdep init
rosdep update

# Source the ROS 2 environment
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Installing Additional Dependencies for Humanoid Robotics

```bash
# Install ros2_control and related packages
sudo apt install -y ros-humble-ros2-control ros-humble-ros2-controllers
sudo apt install -y ros-humble-joint-state-broadcaster ros-humble-velocity-controllers
sudo apt install -y ros-humble-effort-controllers ros-humble-position-controllers

# Install navigation and manipulation packages
sudo apt install -y ros-humble-navigation2 ros-humble-nav2-bringup
sudo apt install -y ros-humble-moveit ros-humble-moveit-ros

# Install simulation packages
sudo apt install -y ros-humble-gazebo-ros2-control ros-humble-gazebo-dev
sudo apt install -y ros-humble-ros-gz ros-humble-ros-gz-sim

# Install visualization tools
sudo apt install -y ros-humble-rviz2 ros-humble-ros2-run
```

## Basic ROS 2 Concepts with Examples

Let's start with a simple example to demonstrate the core ROS 2 concepts. We'll create a simple publisher and subscriber that could be used in a humanoid robot to share sensor data or control commands.

### Creating a ROS 2 Workspace

First, let's create a workspace for our humanoid robot packages:

```bash
# Create the workspace directory
mkdir -p ~/humanoid_ws/src
cd ~/humanoid_ws

# Build the workspace
colcon build --packages-select
```

### Basic Publisher Node

Create a simple publisher node that could simulate publishing joint state information from a humanoid robot:

```python
#!/usr/bin/env python3
# File: ~/humanoid_ws/src/humanoid_bringup/humanoid_bringup/publisher_example.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import math
import time

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        self.publisher_ = self.create_publisher(String, 'robot_status', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Joint positions: [{math.sin(self.i * 0.1)}, {math.cos(self.i * 0.1)}], Counter: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    joint_state_publisher = JointStatePublisher()

    try:
        rclpy.spin(joint_state_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        joint_state_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Basic Subscriber Node

Now, let's create a subscriber that could receive and process the joint state information:

```python
#!/usr/bin/env python3
# File: ~/humanoid_ws/src/humanoid_bringup/humanoid_bringup/subscriber_example.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class JointStateSubscriber(Node):
    def __init__(self):
        super().__init__('joint_state_subscriber')
        self.subscription = self.create_subscription(
            String,
            'robot_status',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'Subscribed to robot status: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    joint_state_subscriber = JointStateSubscriber()

    try:
        rclpy.spin(joint_state_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        joint_state_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Running the Example

To run these nodes:

1. Make the Python files executable:
```bash
chmod +x ~/humanoid_ws/src/humanoid_bringup/humanoid_bringup/publisher_example.py
chmod +x ~/humanoid_ws/src/humanoid_bringup/humanoid_bringup/subscriber_example.py
```

2. Terminal 1 - Run the publisher:
```bash
cd ~/humanoid_ws
source install/setup.bash
python3 src/humanoid_bringup/humanoid_bringup/publisher_example.py
```

3. Terminal 2 - Run the subscriber:
```bash
cd ~/humanoid_ws
source install/setup.bash
python3 src/humanoid_bringup/humanoid_bringup/subscriber_example.py
```

## URDF Basics for Humanoid Robots

URDF (Unified Robot Description Format) is XML-based format used to describe robot models in ROS. For humanoid robots, URDF is essential for defining the kinematic structure, visual representation, and physical properties.

### Basic URDF Structure

Here's a minimal URDF example for a simple humanoid robot with a torso and two legs:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Torso link -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 0.8"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Right hip joint and link -->
  <joint name="right_hip_joint" type="revolute">
    <parent link="torso"/>
    <child link="right_thigh"/>
    <origin xyz="-0.1 -0.1 -0.25" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
  </joint>

  <link name="right_thigh">
    <visual>
      <geometry>
        <box size="0.08 0.08 0.4"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 0.8"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.08 0.08 0.4"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.05"/>
    </inertial>
  </link>

  <!-- Left hip joint and link -->
  <joint name="left_hip_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_thigh"/>
    <origin xyz="-0.1 0.1 -0.25" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
  </joint>

  <link name="left_thigh">
    <visual>
      <geometry>
        <box size="0.08 0.08 0.4"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 0.8"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.08 0.08 0.4"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.05"/>
    </inertial>
  </link>
</robot>
```

## Launch Files for Humanoid Robots

Launch files allow you to start multiple nodes with a single command. Here's an example launch file for our simple humanoid:

```python
# File: ~/humanoid_ws/src/humanoid_bringup/humanoid_bringup/launch/simple_humanoid.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Robot State Publisher node
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': open('/path/to/your/robot.urdf').read()}],
            output='screen'
        ),

        # Joint State Publisher (for simulation)
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        ),

        # Your custom nodes
        Node(
            package='humanoid_bringup',
            executable='publisher_example',
            name='joint_state_publisher_node',
            output='screen'
        ),

        Node(
            package='humanoid_bringup',
            executable='subscriber_example',
            name='joint_state_subscriber_node',
            output='screen'
        ),
    ])
```

## Summary

In this chapter, we've covered the fundamental concepts of ROS 2 that are essential for humanoid robotics:

1. We set up the development environment with ROS 2 Humble
2. We explored the core concepts of nodes, topics, services, and actions
3. We created simple publisher and subscriber examples
4. We introduced URDF for describing robot models
5. We showed how to use launch files to coordinate multiple nodes

These concepts form the foundation for more complex humanoid robot systems. In the next chapter, we'll dive deeper into the rclpy Python client library and create more sophisticated humanoid robot control nodes.

## Exercises

1. Modify the publisher example to publish actual joint position values instead of simulated ones
2. Create a new node that subscribes to joint positions and publishes calculated center of mass
3. Extend the URDF example to include arms and a head
4. Create a launch file that starts both the publisher and subscriber nodes with a single command