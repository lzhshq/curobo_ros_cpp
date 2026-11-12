#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 lzhshq
# SPDX-License-Identifier: Apache-2.0
"""Small PlanPose client for manual testing."""
import argparse

import rclpy
from geometry_msgs.msg import Point, Pose, Quaternion
from rclpy.node import Node
from sensor_msgs.msg import JointState

from curobo_planner_interfaces.srv import PlanPose


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--service', default='/curobo_planner/generate_trajectory')
    parser.add_argument('--target', nargs=7, type=float, required=True,
                        metavar=('X', 'Y', 'Z', 'QX', 'QY', 'QZ', 'QW'))
    parser.add_argument('--start', nargs='+', type=float, default=None,
                        help='Start joint positions; names are taken from the artifact')
    parser.add_argument('--start-names', nargs='+', default=None)
    args = parser.parse_args()

    rclpy.init()
    node = Node('curobo_plan_client')
    client = node.create_client(PlanPose, args.service)
    if not client.wait_for_service(timeout_sec=10.0):
        print(f'service not available: {args.service}')
        return 1

    request = PlanPose.Request()
    x, y, z, qx, qy, qz, qw = args.target
    request.target_pose = Pose(
        position=Point(x=x, y=y, z=z),
        orientation=Quaternion(x=qx, y=qy, z=qz, w=qw),
    )
    if args.start:
        request.start_pose = JointState()
        request.start_pose.position = args.start
        if args.start_names:
            request.start_pose.name = args.start_names

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=30.0)
    response = future.result()
    if response is None:
        print('no response')
        return 1

    print(f'success={response.success} message={response.message}')
    print(f'dt={response.dt} steps={len(response.trajectory)}')
    if response.trajectory:
        print('first=', [round(v, 6) for v in response.trajectory[0].position])
        print('last=', [round(v, 6) for v in response.trajectory[-1].position])

    node.destroy_node()
    rclpy.shutdown()
    return 0 if response.success else 1


if __name__ == '__main__':
    raise SystemExit(main())
