# Chapter 3: ROS 2 Logging Best Practices for Humanoid Robots

## Overview

Logging is critical for humanoid robots, as it enables debugging of complex behaviors, analysis of robot performance, and safety monitoring. In this chapter, we'll explore ROS 2's logging system and how to apply it effectively in humanoid robot applications.

## Learning Objectives

By the end of this chapter, you will be able to:

- Use ROS 2's built-in logging system effectively
- Implement structured logging for humanoid robot systems
- Create custom log levels and filters for different robot subsystems
- Integrate logging with robot state monitoring
- Implement remote logging for distributed humanoid robot systems

## ROS 2 Logging Fundamentals

ROS 2 provides a comprehensive logging system through the `rclpy` and `rclcpp` client libraries. The system supports multiple log levels and allows for both synchronous and asynchronous logging.

### Log Levels

ROS 2 supports the following log levels:

- `DEBUG`: Detailed information for diagnosing problems
- `INFO`: General information about robot operations
- `WARN`: Warning about potential issues
- `ERROR`: Errors that don't prevent the robot from continuing
- `FATAL`: Critical errors that require robot shutdown

### Basic Logging Example

```python
# File: ~/humanoid_ws/src/humanoid_bringup/humanoid_bringup/logging_example.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import math


class LoggingExampleNode(Node):
    def __init__(self):
        super().__init__('logging_example_node')

        # Create a publisher
        self.publisher_ = self.create_publisher(String, 'robot_status', 10)

        # Timer for periodic logging
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.i = 0

        # Log node creation
        self.get_logger().info('Logging example node initialized')

    def timer_callback(self):
        # Different log levels for different types of information
        if self.i % 10 == 0:
            self.get_logger().debug(f'Debug info at iteration {self.i}')

        if self.i % 5 == 0:
            self.get_logger().info(f'Robot status update {self.i}')

        if self.i == 25:
            self.get_logger().warn('This is a warning message')

        # Create and publish message
        msg = String()
        msg.data = f'Robot status: {self.i}'
        self.publisher_.publish(msg)

        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    node = LoggingExampleNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down after keyboard interrupt')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Structured Logging for Humanoid Robots

Structured logging is particularly important for humanoid robots, as it allows for systematic analysis of complex behaviors and performance metrics.

### Custom Log Message Format

```python
# File: ~/humanoid_ws/src/humanoid_bringup/humanoid_bringup/structured_logger.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import json
import time


class StructuredLoggerNode(Node):
    def __init__(self):
        super().__init__('structured_logger_node')

        # Subscribe to joint states
        self.subscription = self.create_subscription(
            Float64MultiArray,
            'joint_states',
            self.joint_state_callback,
            10)

        # Timer for system health logging
        self.timer = self.create_timer(0.1, self.health_callback)

        self.get_logger().info('Structured logger initialized')

    def joint_state_callback(self, msg):
        # Create structured log entry for joint states
        log_data = {
            'timestamp': time.time(),
            'node': self.get_name(),
            'message_type': 'joint_states',
            'joint_positions': [float(pos) for pos in msg.data[:6]],  # First 6 joints
            'joint_velocities': [float(vel) for vel in msg.data[6:12]] if len(msg.data) > 6 else [],
            'sequence': self.get_clock().now().nanoseconds
        }

        # Log as JSON string
        self.get_logger().info(json.dumps(log_data))

    def health_callback(self):
        # Create structured log entry for system health
        health_data = {
            'timestamp': time.time(),
            'node': self.get_name(),
            'message_type': 'system_health',
            'cpu_usage': 15.2,  # Simulated value
            'memory_usage': 45.7,  # Simulated value
            'temperature': 42.3,  # Simulated value
            'battery_level': 87.5,  # Simulated value
            'sequence': self.get_clock().now().nanoseconds
        }

        self.get_logger().info(json.dumps(health_data))


def main(args=None):
    rclpy.init(args=args)
    node = StructuredLoggerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Advanced Logging Patterns for Humanoid Robots

### State-Based Logging

For humanoid robots, it's important to log state transitions and behaviors:

```python
# File: ~/humanoid_ws/src/humanoid_bringup/humanoid_bringup/state_logger.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from enum import Enum


class RobotState(Enum):
    IDLE = "idle"
    WALKING = "walking"
    STANDING = "standing"
    BALANCING = "balancing"
    FALLING = "falling"
    RECOVERING = "recovering"


class StateLoggerNode(Node):
    def __init__(self):
        super().__init__('state_logger_node')

        self.current_state = RobotState.IDLE
        self.previous_state = RobotState.IDLE

        # Subscribe to state changes
        self.state_sub = self.create_subscription(
            String,
            'robot_state',
            self.state_callback,
            10)

        # Log initial state
        self.log_state_transition(self.current_state, "Initial state")

    def state_callback(self, msg):
        try:
            new_state = RobotState(msg.data)
            if new_state != self.current_state:
                self.log_state_transition(new_state, "State change detected")
                self.previous_state = self.current_state
                self.current_state = new_state
        except ValueError:
            self.get_logger().warn(f'Invalid state received: {msg.data}')

    def log_state_transition(self, new_state, reason):
        log_msg = {
            'event': 'state_transition',
            'from_state': self.previous_state.value,
            'to_state': new_state.value,
            'reason': reason,
            'timestamp': self.get_clock().now().nanoseconds
        }

        # Use different log levels based on state transition
        if new_state == RobotState.FALLING:
            self.get_logger().fatal(f'STATE CHANGE: {log_msg}')
        elif new_state in [RobotState.BALANCING, RobotState.RECOVERING]:
            self.get_logger().error(f'STATE CHANGE: {log_msg}')
        else:
            self.get_logger().info(f'State transition: {log_msg}')


def main(args=None):
    rclpy.init(args=args)
    node = StateLoggerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutdown requested')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Performance Logging

Logging performance metrics is crucial for humanoid robots to ensure real-time constraints are met:

```python
# File: ~/humanoid_ws/src/humanoid_bringup/humanoid_bringup/performance_logger.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import time
import statistics


class PerformanceLoggerNode(Node):
    def __init__(self):
        super().__init__('performance_logger')

        # Subscribe to control commands
        self.sub = self.create_subscription(
            Float64MultiArray,
            'joint_commands',
            self.command_callback,
            10)

        # Timer for periodic performance logging
        self.timer = self.create_timer(5.0, self.performance_report)

        # Track performance metrics
        self.processing_times = []
        self.message_counts = 0
        self.start_time = time.time()

    def command_callback(self, msg):
        start_process = time.time()

        # Simulate command processing
        processed_data = [val * 1.1 for val in msg.data]  # Simple processing

        end_process = time.time()
        processing_time = (end_process - start_process) * 1000  # Convert to ms

        # Store processing time
        self.processing_times.append(processing_time)
        self.message_counts += 1

        # Log if processing time exceeds threshold
        if processing_time > 10.0:  # 10ms threshold
            self.get_logger().warn(
                f'High processing time: {processing_time:.2f}ms, '
                f'message size: {len(msg.data)}'
            )

    def performance_report(self):
        if not self.processing_times:
            return

        avg_time = statistics.mean(self.processing_times)
        max_time = max(self.processing_times)
        min_time = min(self.processing_times)

        # Calculate messages per second
        elapsed_time = time.time() - self.start_time
        msg_rate = self.message_counts / elapsed_time if elapsed_time > 0 else 0

        perf_data = {
            'event': 'performance_report',
            'average_processing_time_ms': avg_time,
            'max_processing_time_ms': max_time,
            'min_processing_time_ms': min_time,
            'message_count': self.message_counts,
            'message_rate_hz': msg_rate,
            'processing_window_seconds': 5.0,
            'timestamp': self.get_clock().now().nanoseconds
        }

        self.get_logger().info(f'Performance: {perf_data}')

        # Clear for next reporting window
        self.processing_times.clear()


def main(args=None):
    rclpy.init(args=args)
    node = PerformanceLoggerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Performance logger shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Remote Logging Configuration

For distributed humanoid robot systems, you may need to send logs to a central location:

```yaml
# File: ~/humanoid_ws/src/humanoid_bringup/config/logging_config.yaml
# Logging configuration for humanoid robot system

log_config:
  # Console output settings
  console:
    enabled: true
    level: "INFO"
    format: "[{severity}] [{name}] [{time}] {message}"

  # File output settings
  file:
    enabled: true
    level: "DEBUG"
    path: "/var/log/humanoid_robot/"
    filename: "robot_system.log"
    max_size_bytes: 10485760  # 10MB
    max_files: 5

  # Remote logging settings
  remote:
    enabled: true
    host: "logging-server.local"
    port: 514
    level: "WARN"
    protocol: "udp"

  # Specialized loggers for different subsystems
  subsystems:
    - name: "control"
      level: "DEBUG"
      file: "control.log"
    - name: "perception"
      level: "INFO"
      file: "perception.log"
    - name: "navigation"
      level: "INFO"
      file: "navigation.log"
    - name: "safety"
      level: "FATAL"
      file: "safety.log"
      remote_enabled: true
```

## Integration with Robot Frameworks

Logging should be integrated with other robot frameworks like ros2_control:

```python
# File: ~/humanoid_ws/src/humanoid_bringup/humanoid_bringup/control_logger.py

import rclpy
from rclpy.node import Node
from control_msgs.msg import JointTrajectoryControllerState
from sensor_msgs.msg import JointState
import time


class ControlLoggerNode(Node):
    def __init__(self):
        super().__init__('control_logger')

        # Subscribe to controller state
        self.controller_sub = self.create_subscription(
            JointTrajectoryControllerState,
            '/joint_trajectory_controller/state',
            self.controller_state_callback,
            10)

        # Subscribe to joint states
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)

        # Timer for periodic logging
        self.timer = self.create_timer(1.0, self.periodic_log)

        self.last_controller_update = time.time()
        self.last_joint_update = time.time()

        self.get_logger().info('Control logger initialized')

    def controller_state_callback(self, msg):
        # Log controller state with performance metrics
        state_data = {
            'event': 'controller_state',
            'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            'joint_names': list(msg.joint_names),
            'position_error': self.calculate_error(msg.desired.positions, msg.actual.positions),
            'velocity_error': self.calculate_error(msg.desired.velocities, msg.actual.velocities),
            'effort_error': self.calculate_error(msg.desired.effort, msg.actual.effort),
            'update_interval_ms': (time.time() - self.last_controller_update) * 1000
        }

        self.last_controller_update = time.time()

        # Log based on error levels
        max_pos_error = max(state_data['position_error']) if state_data['position_error'] else 0
        if max_pos_error > 0.1:  # High error threshold
            self.get_logger().error(f'Controller error: {state_data}')
        elif max_pos_error > 0.01:  # Medium error threshold
            self.get_logger().warn(f'Controller error: {state_data}')
        else:
            self.get_logger().debug(f'Controller state: {state_data}')

    def joint_state_callback(self, msg):
        # Log joint state information
        joint_data = {
            'event': 'joint_state',
            'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            'joint_count': len(msg.name),
            'update_interval_ms': (time.time() - self.last_joint_update) * 1000
        }

        self.last_joint_update = time.time()
        self.get_logger().debug(f'Joint state: {joint_data}')

    def periodic_log(self):
        # Log periodic system status
        status = {
            'event': 'system_status',
            'timestamp': self.get_clock().now().nanoseconds,
            'controller_update_rate': 1.0 / (time.time() - self.last_controller_update) if time.time() - self.last_controller_update > 0 else 0,
            'joint_update_rate': 1.0 / (time.time() - self.last_joint_update) if time.time() - self.last_joint_update > 0 else 0,
            'node_healthy': True
        }

        self.get_logger().info(f'System status: {status}')

    def calculate_error(self, desired, actual):
        if len(desired) != len(actual):
            return []
        return [abs(d - a) for d, a in zip(desired, actual)]


def main(args=None):
    rclpy.init(args=args)
    node = ControlLoggerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Control logger shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Best Practices for Humanoid Robot Logging

### 1. Use Appropriate Log Levels

- **DEBUG**: Detailed information for development and debugging
- **INFO**: General operational information
- **WARN**: Potential issues that don't affect operation
- **ERROR**: Problems that may affect robot behavior
- **FATAL**: Critical issues requiring immediate attention

### 2. Include Contextual Information

Always include relevant context in logs:
- Timestamps
- Node names
- Robot state
- Joint positions/values when relevant

### 3. Structure Log Messages

Use consistent, structured formats that can be easily parsed by analysis tools.

### 4. Manage Log Volume

Be mindful of log volume, especially on resource-constrained humanoid robots:
- Use appropriate log levels
- Limit repetitive messages
- Consider log rotation

### 5. Security Considerations

- Don't log sensitive information
- Consider network security for remote logging
- Protect log files from unauthorized access

## Summary

In this chapter, we've covered:

1. ROS 2's built-in logging system and log levels
2. Structured logging for humanoid robot systems
3. State-based and performance logging patterns
4. Remote logging configuration
5. Integration with robot control frameworks
6. Best practices for humanoid robot logging

Proper logging is essential for humanoid robot development, enabling debugging, performance analysis, and safety monitoring. The patterns shown in this chapter will help you implement effective logging in your humanoid robot applications.

## Exercises

1. Create a logging node that monitors and logs the robot's center of mass position
2. Implement a log analysis tool that parses structured logs and generates performance reports
3. Add logging to your existing ROS 2 nodes with appropriate log levels
4. Create a custom logging formatter for humanoid robot applications