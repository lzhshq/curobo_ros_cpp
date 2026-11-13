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
from visualization_msgs.msg import Marker, MarkerArray


class TargetCycle(Node):
    def __init__(self):
        super().__init__('target_cycle')
        self.base_frame = self.declare_parameter('base_frame', 'panda_link0').value
        self.tool_frame = self.declare_parameter('tool_frame', 'panda_hand').value
        self.goal_topic = self.declare_parameter(
            'goal_topic', '/curobo_mpc/goal').value
        self.position_tolerance = float(
            self.declare_parameter('position_tolerance', 0.01).value)
        self.orientation_tolerance = float(
            self.declare_parameter('orientation_tolerance', 0.20).value)
        self.dwell = float(self.declare_parameter('dwell', 0.6).value)
        self.max_hold = float(self.declare_parameter('max_hold', 5.0).value)
        self.period = float(self.declare_parameter('period', 0.10).value)

        # x, y, z, qx, qy, qz, qw in the base frame.
        # Same height Z, symmetric about the base X axis, in +X.
        default_a = [0.30, 0.30, 0.30,
                     -0.988745, 0.007219, -0.149434, 0.001091]
        default_b = [0.30, -0.30, 0.30,
                     -0.988745, 0.007219, -0.149434, 0.001091]
        self.targets = [
            list(self.declare_parameter('target_a', default_a).value),
            list(self.declare_parameter('target_b', default_b).value),
        ]
        if any(len(target) != 7 for target in self.targets):
            raise RuntimeError('target_a and target_b must be 7 numbers')

        self.goal_pub = self.create_publisher(PoseStamped, self.goal_topic, 10)
        self.status_pub = self.create_publisher(String, '~/status', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/curobo_demo/targets', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.index = 0
        self.dwell_start = None
        self.start_time = None
        self.last_publish = None
        self.goal_time = None
        self.create_timer(self.period, self.tick)
        self.publish_markers()
        self.get_logger().info(
            f'target cycle: {self.base_frame} -> {self.tool_frame}, '
            f'tolerance={self.position_tolerance} m, dwell={self.dwell} s')

    def publish_markers(self):
        markers = MarkerArray()
        clear = Marker()
        clear.header.frame_id = self.base_frame
        clear.header.stamp = self.get_clock().now().to_msg()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        for index, target in enumerate(self.targets):
            marker = Marker()
            marker.header.frame_id = self.base_frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'mpc_targets'
            marker.id = index + 1
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = target[0]
            marker.pose.position.y = target[1]
            marker.pose.position.z = target[2]
            marker.scale.x = marker.scale.y = marker.scale.z = 0.04
            marker.color.r = 1.0 if index == 0 else 0.0
            marker.color.g = 0.0 if index == 0 else 1.0
            marker.color.b = 0.0
            marker.color.a = 0.85
            markers.markers.append(marker)
        self.marker_pub.publish(markers)

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
        self.publish_markers()
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
