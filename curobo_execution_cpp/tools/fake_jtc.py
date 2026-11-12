#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 lzhshq
# SPDX-License-Identifier: Apache-2.0
"""Kinematic fake JTC for testing curobo_execution_cpp without hardware.

It is not a physics simulator. It accepts FollowJointTrajectory, interpolates
joint positions, publishes /joint_states, and returns a standard result.
"""
import threading
import time

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState


def to_seconds(duration: Duration) -> float:
    return float(duration.sec) + 1e-9 * float(duration.nanosec)


class FakeJtc(Node):
    def __init__(self) -> None:
        super().__init__('fake_jtc')
        default_joints = [f'joint{i}' for i in range(1, 8)]
        self.joint_names = list(self.declare_parameter('joint_names', default_joints).value)
        self.action_name = str(self.declare_parameter(
            'action', '/position_joint_trajectory_controller/follow_joint_trajectory').value)
        self.rate_hz = float(self.declare_parameter('publish_rate', 100.0).value)
        self.slowdown = float(self.declare_parameter('slowdown', 1.0).value)
        initial = list(self.declare_parameter(
            'initial_positions', [0.0] * len(self.joint_names)).value)
        if len(initial) != len(self.joint_names):
            initial = [0.0] * len(self.joint_names)
        self.q = [float(v) for v in initial]
        self.vel = [0.0] * len(self.joint_names)
        self.lock = threading.Lock()

        self.publisher = self.create_publisher(JointState, '/joint_states', 20)
        self.create_timer(1.0 / max(self.rate_hz, 1.0), self._publish)
        self.server = ActionServer(
            self,
            FollowJointTrajectory,
            self.action_name,
            execute_callback=self._execute,
            goal_callback=lambda _goal: GoalResponse.ACCEPT,
            cancel_callback=lambda _goal: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup(),
        )
        self.get_logger().info(
            f'fake JTC ready: {len(self.joint_names)} joints, action={self.action_name}, '
            f'initial={self.q}')

    def _publish(self) -> None:
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        with self.lock:
            message.name = list(self.joint_names)
            message.position = list(self.q)
            message.velocity = list(self.vel)
        self.publisher.publish(message)

    def _result(self, code: int, text: str) -> FollowJointTrajectory.Result:
        return FollowJointTrajectory.Result(error_code=code, error_string=text)

    def _execute(self, goal_handle):
        trajectory = goal_handle.request.trajectory
        if not trajectory.points:
            goal_handle.abort()
            return self._result(-1, 'empty trajectory')
        if not trajectory.joint_names:
            goal_handle.abort()
            return self._result(-1, 'trajectory has no joint_names')

        try:
            target_index = [trajectory.joint_names.index(name) for name in self.joint_names]
        except ValueError as exception:
            goal_handle.abort()
            return self._result(-2, f'joint mismatch: {exception}')

        points = []
        for point in trajectory.points:
            if len(point.positions) != len(trajectory.joint_names):
                goal_handle.abort()
                return self._result(-1, 'point positions do not match joint_names')
            points.append((
                to_seconds(point.time_from_start),
                [float(point.positions[index]) for index in target_index],
            ))
        points.sort(key=lambda item: item[0])

        if len(points) == 1:
            with self.lock:
                self.q = list(points[0][1])
                self.vel = [0.0] * len(self.q)
            goal_handle.succeed()
            return self._result(0, 'single point applied')

        self.get_logger().info(
            f'executing {len(points)} points / {points[-1][0]:.3f}s')
        start = time.monotonic()
        previous = list(self.q)
        while True:
            if goal_handle.is_cancel_requested:
                with self.lock:
                    self.vel = [0.0] * len(self.q)
                goal_handle.canceled()
                return self._result(-1, 'canceled')

            elapsed = (time.monotonic() - start) / self.slowdown
            if elapsed >= points[-1][0]:
                with self.lock:
                    self.q = list(points[-1][1])
                    self.vel = [0.0] * len(self.q)
                break

            hi = 1
            while hi < len(points) and points[hi][0] < elapsed:
                hi += 1
            lo = hi - 1
            span = points[hi][0] - points[lo][0]
            ratio = 0.0 if span <= 1e-9 else (elapsed - points[lo][0]) / span
            current = [
                a + ratio * (b - a)
                for a, b in zip(points[lo][1], points[hi][1])
            ]
            with self.lock:
                self.q = current
                self.vel = [
                    (b - a) / max(span, 1e-6)
                    for a, b in zip(points[lo][1], points[hi][1])
                ]
            time.sleep(0.01)

        with self.lock:
            self.vel = [0.0] * len(self.q)
        goal_handle.succeed()
        return self._result(0, 'trajectory completed')


def main() -> None:
    rclpy.init()
    node = FakeJtc()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
