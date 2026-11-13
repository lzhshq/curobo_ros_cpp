#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 lzhshq
# SPDX-License-Identifier: Apache-2.0
"""Alternate two Cartesian goals when the tool reaches the current one."""

import math

from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
import rclpy
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener


class TargetCycle(Node):
    def __init__(self):
        super().__init__('target_cycle')
        self.base_frame = self.declare_parameter('base_frame', 'panda_link0').value
        self.tool_frame = self.declare_parameter('tool_frame', 'panda_hand').value
        self.goal_topic = self.declare_parameter(
            'goal_topic', '/curobo_mpc/goal').value
        self.position_tolerance = float(
            self.declare_parameter('position_tolerance', 0.02).value)
        self.orientation_tolerance = float(
            self.declare_parameter('orientation_tolerance', 0.20).value)
        self.dwell = float(self.declare_parameter('dwell', 0.6).value)
        self.max_hold = float(self.declare_parameter('max_hold', 5.0).value)
        self.period = float(self.declare_parameter('period', 0.10).value)

        # x, y, z, qx, qy, qz, qw in the base frame.
        default_a = [0.11332, 0.18179, 0.67355,
                     0.00665, -0.94516, -0.32638, 0.01008]
        default_b = [0.10543, -0.12509, 0.59443,
                     0.07429, 0.95883, -0.14682, 0.23145]
        self.targets = [
            list(self.declare_parameter('target_a', default_a).value),
            list(self.declare_parameter('target_b', default_b).value),
        ]
        if any(len(target) != 7 for target in self.targets):
            raise RuntimeError('target_a and target_b must be 7 numbers')

        self.goal_pub = self.create_publisher(PoseStamped, self.goal_topic, 10)
        self.status_pub = self.create_publisher(String, '~/status', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.index = 0
        self.dwell_start = None
        self.start_time = None
        self.last_publish = None
        self.goal_time = None
        self.create_timer(self.period, self.tick)
        self.get_logger().info(
            f'target cycle: {self.base_frame} -> {self.tool_frame}, '
            f'tolerance={self.position_tolerance} m, dwell={self.dwell} s')

    def publish_target(self):
        target = self.targets[self.index]
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.base_frame
        message.pose.position.x = target[0]
        message.pose.position.y = target[1]
        message.pose.position.z = target[2]
        message.pose.orientation.x = target[3]
        message.pose.orientation.y = target[4]
        message.pose.orientation.z = target[5]
        message.pose.orientation.w = target[6]
        self.goal_pub.publish(message)
        self.last_publish = self.get_clock().now()
        self.goal_time = self.last_publish
        status = f'goal {chr(ord("A") + self.index)}'
        self.status_pub.publish(String(data=status))
        self.get_logger().info(status)

    def tick(self):
        now = self.get_clock().now()
        if self.start_time is None:
            self.start_time = now
            self.publish_target()
            return
        # Repeat the first goal briefly so a late DDS discovery does not lose it.
        if (now - self.start_time) < Duration(seconds=1.5):
            if (now - self.last_publish) > Duration(seconds=0.3):
                self.publish_target()
            return
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame, self.tool_frame, rclpy.time.Time())
        except Exception as exception:  # TF may not have arrived yet
            self.get_logger().debug(f'TF unavailable: {exception}', throttle_duration_sec=2.0)
            return

        target = self.targets[self.index]
        dx = transform.transform.translation.x - target[0]
        dy = transform.transform.translation.y - target[1]
        dz = transform.transform.translation.z - target[2]
        position_error = math.sqrt(dx * dx + dy * dy + dz * dz)

        q = transform.transform.rotation
        qx, qy, qz, qw = target[3], target[4], target[5], target[6]
        dot = abs(q.x * qx + q.y * qy + q.z * qz + q.w * qw)
        dot = min(1.0, max(-1.0, dot))
        orientation_error = 2.0 * math.acos(dot)

        if position_error > self.position_tolerance or orientation_error > self.orientation_tolerance:
            self.dwell_start = None
            if (now - self.goal_time) > Duration(seconds=self.max_hold):
                self.index = 1 - self.index
                self.dwell_start = None
                self.publish_target()
            return
        if self.dwell_start is None:
            self.dwell_start = now
            return
        if (now - self.dwell_start) < Duration(seconds=self.dwell):
            return

        self.index = 1 - self.index
        self.dwell_start = None
        self.publish_target()


def main():
    rclpy.init()
    node = TargetCycle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
