#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 lzhshq
# SPDX-License-Identifier: Apache-2.0
"""Start an MPC action and stream a circle through /unified_planner/mpc_goal."""
import math

import rclpy
from action_msgs.msg import GoalStatus
from curobo_msgs.action import SendTrajectory
from geometry_msgs.msg import Pose
from rclpy.action import ActionClient
from rclpy.node import Node


class UnifiedCircleTarget(Node):
    def __init__(self):
        super().__init__('unified_circle_target')
        self.x = float(self.declare_parameter('center_x', 0.161283).value)
        self.y = float(self.declare_parameter('center_y', 0.0).value)
        self.z = float(self.declare_parameter('center_z', 0.637).value)
        self.radius = float(self.declare_parameter('radius', 0.04).value)
        self.period = float(self.declare_parameter('period', 10.0).value)
        self.rate = float(self.declare_parameter('rate', 10.0).value)
        self.orientation = [
            float(v) for v in self.declare_parameter(
                'orientation_xyzw',
                [-0.988745, 0.007219, -0.149434, 0.001091]).value]
        self.goal_pub = self.create_publisher(Pose, '/unified_planner/mpc_goal', 10)
        self.client = ActionClient(self, SendTrajectory, '/unified_planner/execute_trajectory')
        self.start = self.get_clock().now()
        self.sent = False
        self.create_timer(1.0 / max(self.rate, 0.1), self.tick)
        self.create_timer(0.5, self.ensure_action)
        self.get_logger().info(
            f'unified circle: center=({self.x},{self.y},{self.z}) '
            f'radius={self.radius} period={self.period}s')

    def pose(self, angle):
        pose = Pose()
        pose.position.x = self.x
        pose.position.y = self.y + self.radius * math.cos(angle)
        pose.position.z = self.z + self.radius * math.sin(angle)
        pose.orientation.x = self.orientation[0]
        pose.orientation.y = self.orientation[1]
        pose.orientation.z = self.orientation[2]
        pose.orientation.w = self.orientation[3]
        return pose

    def ensure_action(self):
        if self.sent:
            return
        if not self.client.server_is_ready():
            return
        goal = SendTrajectory.Goal()
        goal.target_pose = self.pose(0.0)
        self.sent = True
        future = self.client.send_goal_async(goal)
        future.add_done_callback(self.goal_response)

    def goal_response(self, future):
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error('MPC action rejected')
            self.sent = False
            return
        self.get_logger().info('MPC action accepted')

    def tick(self):
        if not self.sent:
            return
        elapsed = (self.get_clock().now() - self.start).nanoseconds * 1e-9
        angle = 2.0 * math.pi * elapsed / max(self.period, 1e-3)
        self.goal_pub.publish(self.pose(angle))


def main():
    rclpy.init()
    node = UnifiedCircleTarget()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
