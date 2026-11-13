#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 lzhshq
# SPDX-License-Identifier: Apache-2.0
"""Publish a circular Cartesian target for MPC tracking.

The circle lies in the base YZ plane at fixed X, so the arm moves in a
front-facing vertical circle.
"""
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


class CircleTarget(Node):
    def __init__(self):
        super().__init__('circle_target')
        self.frame_id = self.declare_parameter('frame_id', 'panda_link0').value
        self.x = float(self.declare_parameter('center_x', 0.161283).value)
        self.y = float(self.declare_parameter('center_y', 0.0).value)
        self.z = float(self.declare_parameter('center_z', 0.637).value)
        self.radius = float(self.declare_parameter('radius', 0.04).value)
        self.period = float(self.declare_parameter('period', 5.0).value)
        self.rate = float(self.declare_parameter('rate', 10.0).value)
        self.orientation = [
            float(value) for value in self.declare_parameter(
                'orientation_xyzw',
                [-0.988745, 0.007219, -0.149434, 0.001091]).value]
        self.topic = self.declare_parameter('goal_topic', '/curobo_mpc/live_goal').value
        self.publisher = self.create_publisher(PoseStamped, self.topic, 10)
        self.start = self.get_clock().now()
        self.create_timer(1.0 / max(self.rate, 0.1), self.tick)
        self.get_logger().info(
            f'circle target: center=({self.x},{self.y},{self.z}) '
            f'radius={self.radius} period={self.period}s rate={self.rate}Hz')

    def tick(self):
        elapsed = (self.get_clock().now() - self.start).nanoseconds * 1e-9
        angle = 2.0 * math.pi * elapsed / max(self.period, 1e-3)
        message = PoseStamped()
        message.header.frame_id = self.frame_id
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.position.x = self.x
        message.pose.position.y = self.y + self.radius * math.cos(angle)
        message.pose.position.z = self.z + self.radius * math.sin(angle)
        message.pose.orientation.x = self.orientation[0]
        message.pose.orientation.y = self.orientation[1]
        message.pose.orientation.z = self.orientation[2]
        message.pose.orientation.w = self.orientation[3]
        self.publisher.publish(message)


def main():
    rclpy.init()
    node = CircleTarget()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
