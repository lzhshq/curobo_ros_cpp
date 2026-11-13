#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 lzhshq
# SPDX-License-Identifier: Apache-2.0
"""Headless regression for the Franka MPC demo.

Run against a stack started with autostart_cycle:=false:

    ros2 launch curobo_mpc_cpp franka_bat_demo.launch.py autostart_cycle:=false
    ros2 run curobo_mpc_cpp mpc_regression
"""
import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

Q = [-0.988745, 0.007219, -0.149434, 0.001091]
TARGETS = [
    ('home', [0.161283, 0.0, 0.636847, *Q]),
    ('small_y', [0.161283, 0.02, 0.645, *Q]),
    ('a', [0.30, 0.30, 0.30, *Q]),
    ('b', [0.30, -0.30, 0.30, *Q]),
]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--timeout', type=float, default=8.0)
    parser.add_argument('--tolerance', type=float, default=0.02)
    args = parser.parse_args()

    rclpy.init()
    node = Node('mpc_regression')
    publisher = node.create_publisher(PoseStamped, '/curobo_mpc/goal', 10)
    status = ['?']
    node.create_subscription(
        String, '/curobo_mpc/status',
        lambda message: status.__setitem__(0, message.data), 10)
    tf_buffer = Buffer()
    TransformListener(tf_buffer, node)

    failures = 0
    for name, target in TARGETS:
        message = PoseStamped()
        message.header.frame_id = 'panda_link0'
        message.pose.position.x = target[0]
        message.pose.position.y = target[1]
        message.pose.position.z = target[2]
        message.pose.orientation.x = target[3]
        message.pose.orientation.y = target[4]
        message.pose.orientation.z = target[5]
        message.pose.orientation.w = target[6]
        publisher.publish(message)

        deadline = time.monotonic() + args.timeout
        error = float('inf')
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            try:
                transform = tf_buffer.lookup_transform(
                    'panda_link0', 'panda_hand', rclpy.time.Time(),
                    timeout=Duration(seconds=0.05))
            except Exception:
                continue
            position = transform.transform.translation
            error = math.sqrt(
                (position.x - target[0]) ** 2 +
                (position.y - target[1]) ** 2 +
                (position.z - target[2]) ** 2)
            if error <= args.tolerance:
                break

        passed = error <= args.tolerance
        print(f'{name}: error={error * 100:.2f} cm '
              f'{"PASS" if passed else "FAIL"} status={status[0]}')
        failures += 0 if passed else 1

    node.destroy_node()
    rclpy.shutdown()
    return 0 if failures == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
