# Chapter 2: The Robotic Nervous System (ROS 2) - rclpy Deep Dive

## Overview

In this chapter, we'll dive deeper into the rclpy Python client library, which is the Python API for ROS 2. We'll explore advanced concepts including custom message types, services, actions, and how to structure humanoid robot control nodes effectively.

## Learning Objectives

By the end of this chapter, you will be able to:

- Create and use custom message types for humanoid robot applications
- Implement and use ROS 2 services for robot configuration
- Work with ROS 2 actions for long-running humanoid robot tasks
- Structure complex humanoid robot nodes with proper lifecycle management
- Implement multi-threading in ROS 2 nodes for humanoid robot control
- Use parameters effectively in humanoid robot nodes

## Custom Message Types

While ROS 2 provides many standard message types, humanoid robots often require custom messages for specific applications. Let's create a custom message for humanoid robot state information.

### Creating a Custom Message Package

First, create a package for custom messages:

```bash
# In your workspace
cd ~/humanoid_ws/src
ros2 pkg create --build-type ament_python humanoid_msgs
```

Then create the message definition files:

```bash
# Create directories for different message types
mkdir -p ~/humanoid_ws/src/humanoid_msgs/humanoid_msgs/msg
mkdir -p ~/humanoid_ws/src/humanoid_msgs/humanoid_msgs/srv
mkdir -p ~/humanoid_ws/src/humanoid_msgs/humanoid_msgs/action
```

### Humanoid Robot State Message

Create a custom message for humanoid robot state:

```python
# File: ~/humanoid_ws/src/humanoid_msgs/humanoid_msgs/msg/HumanoidState.msg
# HumanoidRobotState.msg
float64[] joint_positions
float64[] joint_velocities
float64[] joint_efforts
float64[3] center_of_mass
float64[4] orientation  # quaternion (x, y, z, w)
float64[3] linear_velocity
float64[3] angular_velocity
string[] joint_names
bool in_contact  # whether the robot is in contact with ground
bool is_balanced  # whether the robot is balanced
```

### Humanoid Robot Command Message

Create a command message for controlling the humanoid:

```python
# File: ~/humanoid_ws/src/humanoid_msgs/humanoid_msgs/msg/HumanoidCommand.msg
# HumanoidCommand.msg
float64[] joint_positions
float64[] joint_velocities
float64[] joint_efforts
float64[3] target_com  # target center of mass
float64[3] target_velocity
string[] joint_names
uint8[] modes  # control modes for each joint
```

### Package Configuration

Update the package configuration files:

```python
# File: ~/humanoid_ws/src/humanoid_msgs/setup.py
from setuptools import find_packages, setup

package_name = 'humanoid_msgs'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Custom messages for humanoid robotics',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
```

```xml
<!-- File: ~/humanoid_ws/src/humanoid_msgs/package.xml -->
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>humanoid_msgs</name>
  <version>0.0.0</version>
  <description>Custom messages for humanoid robotics</description>
  <maintainer email="your_email@example.com">your_name</maintainer>
  <license>Apache License 2.0</license>

  <buildtool_depend>ament_python</buildtool_depend>

  <depend>std_msgs</depend>
  <depend>builtin_interfaces</depend>

  <test_depend>pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

### Building Custom Messages

To build the custom messages, you need to modify the setup files to include the message generation:

```python
# File: ~/humanoid_ws/src/humanoid_msgs/setup.py (updated)
from setuptools import find_packages, setup

package_name = 'humanoid_msgs'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Custom messages for humanoid robotics',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
    package_dir={'humanoid_msgs': 'humanoid_msgs'},
    package_data={'humanoid_msgs': ['msg/*.msg']},
)
```

## Services for Humanoid Robot Configuration

Services provide synchronous request/response communication, which is useful for configuring humanoid robot parameters or requesting specific information.

### Creating a Configuration Service

```python
# File: ~/humanoid_ws/src/humanoid_msgs/humanoid_msgs/srv/RobotConfiguration.srv
# Request
string parameter_name
string parameter_value

# Response
bool success
string message
```

### Service Server Implementation

```python
# File: ~/humanoid_ws/src/humanoid_bringup/humanoid_bringup/config_server.py

import rclpy
from rclpy.node import Node
from humanoid_msgs.srv import RobotConfiguration


class ConfigServer(Node):
    def __init__(self):
        super().__init__('config_server')
        self.srv = self.create_service(
            RobotConfiguration,
            'set_robot_configuration',
            self.set_config_callback
        )

        # Initialize configuration parameters
        self.config_params = {
            'walking_speed': 0.5,
            'max_joint_velocity': 1.0,
            'balance_threshold': 0.05
        }

        self.get_logger().info('Configuration server is ready')

    def set_config_callback(self, request, response):
        param_name = request.parameter_name
        param_value = request.parameter_value

        if param_name in self.config_params:
            try:
                # Convert string value to appropriate type based on parameter
                if param_name in ['walking_speed', 'balance_threshold', 'max_joint_velocity']:
                    self.config_params[param_name] = float(param_value)
                else:
                    self.config_params[param_name] = param_value

                response.success = True
                response.message = f'Parameter {param_name} set to {param_value}'
                self.get_logger().info(f'Set {param_name} to {param_value}')
            except ValueError:
                response.success = False
                response.message = f'Invalid value for {param_name}: {param_value}'
        else:
            response.success = False
            response.message = f'Unknown parameter: {param_name}

        return response


def main(args=None):
    rclpy.init(args=args)
    config_server = ConfigServer()

    try:
        rclpy.spin(config_server)
    except KeyboardInterrupt:
        pass
    finally:
        config_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Service Client Implementation

```python
# File: ~/humanoid_ws/src/humanoid_bringup/humanoid_bringup/config_client.py

import rclpy
from rclpy.node import Node
from humanoid_msgs.srv import RobotConfiguration


class ConfigClient(Node):
    def __init__(self):
        super().__init__('config_client')
        self.cli = self.create_client(RobotConfiguration, 'set_robot_configuration')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = RobotConfiguration.Request()

    def send_request(self, param_name, param_value):
        self.req.parameter_name = param_name
        self.req.parameter_value = param_value
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)
    config_client = ConfigClient()

    # Example: Set walking speed
    response = config_client.send_request('walking_speed', '0.7')
    if response:
        if response.success:
            print(f'Success: {response.message}')
        else:
            print(f'Error: {response.message}')
    else:
        print('Service call failed')

    config_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Actions for Long-Running Humanoid Tasks

Actions are perfect for long-running humanoid robot tasks like walking, manipulation, or navigation that provide feedback during execution.

### Creating a Walking Action

```python
# File: ~/humanoid_ws/src/humanoid_msgs/humanoid_msgs/action/Walk.action
# Goal
float64[3] target_position  # x, y, theta
float64 speed
bool avoid_obstacles

# Result
bool success
string message
float64[3] final_position  # actual final position

# Feedback
float64[3] current_position
float64 progress  # 0.0 to 1.0
bool is_balanced
```

### Action Server Implementation

```python
# File: ~/humanoid_ws/src/humanoid_bringup/humanoid_bringup/walk_server.py

import time
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from humanoid_msgs.action import Walk


class WalkServer(Node):
    def __init__(self):
        super().__init__('walk_server')
        self._action_server = ActionServer(
            self,
            Walk,
            'walk_to_goal',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)
        self.get_logger().info('Walk action server is ready')

    def goal_callback(self, goal_request):
        # Accept all goals
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        # Accept all cancel requests
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        # Get goal parameters
        target_position = goal_handle.request.target_position
        speed = goal_handle.request.speed
        avoid_obstacles = goal_handle.request.avoid_obstacles

        # Initialize feedback
        feedback_msg = Walk.Feedback()
        feedback_msg.current_position = [0.0, 0.0, 0.0]  # Starting position
        feedback_msg.progress = 0.0
        feedback_msg.is_balanced = True

        # Simulate walking to goal
        for i in range(10):  # Simulate 10 steps
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Walk.Result()

            # Simulate movement
            feedback_msg.current_position[0] += target_position[0] * 0.1
            feedback_msg.current_position[1] += target_position[1] * 0.1
            feedback_msg.progress = (i + 1) * 0.1
            feedback_msg.is_balanced = True  # Simulated balance status

            # Publish feedback
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Feedback: {feedback_msg.progress:.2f} progress')

            # Sleep to simulate walking
            time.sleep(1.0)

        # Populate result
        result = Walk.Result()
        result.success = True
        result.message = 'Successfully reached target position'
        result.final_position = target_position

        goal_handle.succeed()
        self.get_logger().info('Goal succeeded')

        return result


def main(args=None):
    rclpy.init(args=args)
    walk_server = WalkServer()

    try:
        rclpy.spin(walk_server)
    except KeyboardInterrupt:
        pass
    finally:
        walk_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Action Client Implementation

```python
# File: ~/humanoid_ws/src/humanoid_bringup/humanoid_bringup/walk_client.py

import time
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from humanoid_msgs.action import Walk


class WalkClient(Node):
    def __init__(self):
        super().__init__('walk_client')
        self._action_client = ActionClient(self, Walk, 'walk_to_goal')

    def send_goal(self, target_position, speed=0.5, avoid_obstacles=False):
        goal_msg = Walk.Goal()
        goal_msg.target_position = target_position
        goal_msg.speed = speed
        goal_msg.avoid_obstacles = avoid_obstacles

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Feedback received: {feedback.progress:.2f} progress, '
            f'position: [{feedback.current_position[0]:.2f}, {feedback.current_position[1]:.2f}]')

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.success}, {result.message}')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    action_client = WalkClient()

    # Send a goal to walk to position [1.0, 1.0, 0.0]
    action_client.send_goal([1.0, 1.0, 0.0], speed=0.5)

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
```

## Advanced Node Patterns for Humanoid Robots

### Lifecycle Nodes

For humanoid robots that need to go through different operational states (startup, calibration, operation, shutdown), lifecycle nodes are useful:

```python
# File: ~/humanoid_ws/src/humanoid_bringup/humanoid_bringup/lifecycle_humanoid_node.py

import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.lifecycle import Publisher
from std_msgs.msg import String


class HumanoidLifecycleNode(LifecycleNode):
    def __init__(self):
        super().__init__('humanoid_lifecycle_node')
        self.get_logger().info('Humanoid Lifecycle Node created')

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Configuring node...')

        # Create publisher
        self.pub = self.create_publisher(String, 'robot_status', 10)

        # Initialize robot systems
        self.get_logger().info('Robot systems initialized')

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Activating node...')

        # Activate publisher
        self.pub.on_activate()

        # Start robot operations
        self.timer = self.create_timer(1.0, self.timer_callback)

        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Deactivating node...')

        # Deactivate publisher
        self.pub.on_deactivate()

        # Stop timer
        self.timer.destroy()

        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Cleaning up node...')

        # Destroy publisher
        del self.pub

        return TransitionCallbackReturn.SUCCESS

    def timer_callback(self):
        msg = String()
        msg.data = 'Robot is operational'
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = HumanoidLifecycleNode()

    # Transition through states
    node.trigger_configure()
    node.trigger_activate()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.trigger_deactivate()
        node.trigger_cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Multi-threading in ROS 2 Nodes

For humanoid robots that need to handle multiple tasks simultaneously (e.g., control, perception, communication), multi-threading can be useful:

```python
# File: ~/humanoid_ws/src/humanoid_bringup/humanoid_bringup/multithreaded_controller.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import time
import math


class MultithreadedController(Node):
    def __init__(self):
        super().__init__('multithreaded_controller')

        # Create publishers for different subsystems
        self.status_pub = self.create_publisher(String, 'robot_status', 10)
        self.control_pub = self.create_publisher(String, 'control_commands', 10)

        # Shared data between threads
        self.joint_positions = [0.0] * 2  # Simulated joint positions
        self.lock = threading.Lock()

        # Start control thread
        self.control_thread = threading.Thread(target=self.control_loop)
        self.control_thread.daemon = True
        self.control_thread.start()

        # Start perception thread
        self.perception_thread = threading.Thread(target=self.perception_loop)
        self.perception_thread.daemon = True
        self.perception_thread.start()

        # Timer for status updates
        self.timer = self.create_timer(0.1, self.status_callback)

    def control_loop(self):
        """Control thread for joint position updates"""
        while rclpy.ok():
            with self.lock:
                # Update joint positions (simulated control)
                self.joint_positions[0] = math.sin(time.time() * 0.5)
                self.joint_positions[1] = math.cos(time.time() * 0.3)

            time.sleep(0.01)  # Control loop at 100Hz

    def perception_loop(self):
        """Perception thread for sensor processing"""
        while rclpy.ok():
            # Simulate perception processing
            with self.lock:
                pos = self.joint_positions[:]

            # Publish control commands based on "perception"
            cmd_msg = String()
            cmd_msg.data = f'Perception-based command for joints: {pos}'
            self.control_pub.publish(cmd_msg)

            time.sleep(0.05)  # Perception loop at 20Hz

    def status_callback(self):
        """Main thread callback for status publishing"""
        with self.lock:
            pos = self.joint_positions[:]

        status_msg = String()
        status_msg.data = f'Joints: [{pos[0]:.3f}, {pos[1]:.3f}], Status: OK'
        self.status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    controller = MultithreadedController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Parameters in Humanoid Robot Nodes

Parameters allow runtime configuration of humanoid robot behavior:

```python
# File: ~/humanoid_ws/src/humanoid_bringup/humanoid_bringup/parameterized_controller.py

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Float64MultiArray
import math
from rcl_interfaces.msg import SetParametersResult


class ParameterizedController(Node):
    def __init__(self):
        super().__init__('parameterized_controller')

        # Declare parameters with defaults
        self.declare_parameter('walking_frequency', 0.5)
        self.declare_parameter('step_height', 0.05)
        self.declare_parameter('max_velocity', 1.0)
        self.declare_parameter('control_mode', 'position')

        # Create publisher
        self.cmd_pub = self.create_publisher(Float64MultiArray, 'joint_commands', 10)

        # Get parameter values
        self.frequency = self.get_parameter('walking_frequency').value
        self.step_height = self.get_parameter('step_height').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.control_mode = self.get_parameter('control_mode').value

        self.get_logger().info(
            f'Controller initialized with: freq={self.frequency}, '
            f'step_height={self.step_height}, max_vel={self.max_velocity}, '
            f'control_mode={self.control_mode}'
        )

        # Set up parameter callback
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Timer for control loop
        self.time = 0.0
        self.timer = self.create_timer(0.01, self.control_callback)  # 100Hz

    def parameter_callback(self, params):
        """Callback for parameter changes"""
        for param in params:
            if param.name == 'walking_frequency' and param.type_ == Parameter.Type.DOUBLE:
                self.frequency = param.value
                self.get_logger().info(f'Updated walking frequency to {self.frequency}')
            elif param.name == 'step_height' and param.type_ == Parameter.Type.DOUBLE:
                self.step_height = param.value
                self.get_logger().info(f'Updated step height to {self.step_height}')
            elif param.name == 'max_velocity' and param.type_ == Parameter.Type.DOUBLE:
                self.max_velocity = param.value
                self.get_logger().info(f'Updated max velocity to {self.max_velocity}')
            elif param.name == 'control_mode' and param.type_ == Parameter.Type.STRING:
                self.control_mode = param.value
                self.get_logger().info(f'Updated control mode to {self.control_mode}')

        return SetParametersResult(successful=True)

    def control_callback(self):
        """Main control loop"""
        self.time += 0.01

        # Generate walking pattern
        left_leg = self.step_height * math.sin(2 * math.pi * self.frequency * self.time)
        right_leg = self.step_height * math.sin(2 * math.pi * self.frequency * self.time + math.pi)

        # Create command message
        cmd_msg = Float64MultiArray()
        cmd_msg.data = [left_leg, right_leg, 0.0, 0.0]  # 4 joints: left leg, right leg, left arm, right arm

        self.cmd_pub.publish(cmd_msg)


def main(args=None):
    rclpy.init(args=args)
    controller = ParameterizedController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Summary

In this chapter, we've explored advanced ROS 2 concepts essential for humanoid robotics:

1. We created custom message types for humanoid robot state and commands
2. We implemented services for robot configuration
3. We worked with actions for long-running tasks like walking
4. We explored lifecycle nodes for managing robot states
5. We implemented multi-threading for concurrent operations
6. We used parameters for runtime configuration

These advanced concepts allow you to build sophisticated humanoid robot control systems that can handle complex behaviors and interactions with the environment.

## Exercises

1. Create a custom action for humanoid manipulation tasks (e.g., picking up an object)
2. Implement a parameter server that allows remote configuration of walking parameters
3. Create a lifecycle node that manages the complete startup sequence for a humanoid robot
4. Implement a multi-threaded perception node that processes camera and IMU data simultaneously