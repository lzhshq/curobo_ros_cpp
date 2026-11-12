#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 lzhshq
# SPDX-License-Identifier: Apache-2.0
"""Minimal ForwardCommandController plant for FPC adapter tests.

It subscribes to a Float64MultiArray position command and republishes the last
commanded positions as /joint_states. This is kinematic, not a physics engine.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class FakeFpc(Node):
    def __init__(self):
        super().__init__('fake_fpc')
        self.joint_names = list(self.declare_parameter(
            'joint_names',
            ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4',
             'panda_joint5', 'panda_joint6', 'panda_joint7'],
        ).value)
        initial = list(self.declare_parameter(
            'initial_positions', [0.0] * len(self.joint_names)).value)
        if len(initial) != len(self.joint_names):
            raise RuntimeError('initial_positions must match joint_names')
        self.positions = initial
        command_topic = self.declare_parameter(
            'command_topic', '/position_forward_controller/commands').value
        rate = self.declare_parameter('publish_rate', 100.0).value

        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.create_subscription(
            Float64MultiArray, command_topic, self.command_callback, 10)
        self.create_timer(1.0 / max(rate, 1.0), self.publish_state)
        self.get_logger().info(
            f'fake FPC ready: {len(self.joint_names)} joints, '
            f'command={command_topic}')

    def command_callback(self, message: Float64MultiArray) -> None:
        if len(message.data) != len(self.joint_names):
            self.get_logger().warning(
                f'ignoring command with {len(message.data)} values, '
                f'expected {len(self.joint_names)}')
            return
        self.positions = list(message.data)

    def publish_state(self) -> None:
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(self.joint_names)
        message.position = list(self.positions)
        self.publisher.publish(message)


def main() -> None:
    rclpy.init()
    node = FakeFpc()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
