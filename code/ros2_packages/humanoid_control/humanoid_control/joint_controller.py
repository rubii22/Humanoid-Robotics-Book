# Copyright 2025 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Joint controller for humanoid robot.

This node controls the joints of a humanoid robot, managing position, velocity,
and effort control for each joint.
"""

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState


class JointController(Node):

    def __init__(self):
        super().__init__('joint_controller')

        # Publisher for joint commands
        self.joint_cmd_publisher = self.create_publisher(
            Float64MultiArray,
            '/joint_group_position_controller/commands',
            10
        )

        # Subscriber for joint states
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Timer for control loop
        timer_period = 0.05  # 20 Hz
        self.timer = self.create_timer(timer_period, self.control_loop)

        self.get_logger().info('Joint controller initialized')

    def joint_state_callback(self, msg):
        # Process joint state information
        self.get_logger().debug(f'Received joint states: {len(msg.name)} joints')

    def control_loop(self):
        # Publish joint commands
        cmd_msg = Float64MultiArray()
        cmd_msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Example joint positions
        self.joint_cmd_publisher.publish(cmd_msg)


def main(args=None):
    rclpy.init(args=args)

    joint_controller = JointController()

    try:
        rclpy.spin(joint_controller)
    except KeyboardInterrupt:
        pass
    finally:
        joint_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()