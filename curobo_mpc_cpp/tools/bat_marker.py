#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 lzhshq
# SPDX-License-Identifier: Apache-2.0
"""RViz interactive marker for a bat-like obstacle.

The bat can be dragged along Z, or moved automatically along Z with
``auto_motion=true``. World X/Y are fixed. The visual cylinder's long axis is X.
"""

import math

import time

import rclpy
from geometry_msgs.msg import PoseStamped
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from rclpy.node import Node
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker


class BatMarker(Node):
    def __init__(self):
        super().__init__('bat_marker')
        self.frame_id = self.declare_parameter('frame_id', 'panda_link0').value
        self.fixed_x = float(self.declare_parameter('x', 0.30).value)
        self.initial_y = float(self.declare_parameter('y', 0.0).value)
        self.initial_z = float(self.declare_parameter('z', 0.30).value)
        self.length = float(self.declare_parameter('length', 0.4).value)
        self.radius = float(self.declare_parameter('radius', 0.03).value)
        self.auto_motion = bool(self.declare_parameter('auto_motion', False).value)
        self.motion_amplitude = float(
            self.declare_parameter('motion_amplitude', 0.10).value)
        self.motion_frequency = float(
            self.declare_parameter('motion_frequency', 0.1).value)
        topic = self.declare_parameter(
            'pose_topic', '/curobo_demo/bat_pose').value

        self.pose_pub = self.create_publisher(PoseStamped, topic, 10)
        self.server = InteractiveMarkerServer(self, 'curobo_demo/bat')

        marker = InteractiveMarker()
        marker.header.frame_id = self.frame_id
        marker.name = 'bat'
        marker.description = 'bat: automatic Z' if self.auto_motion else 'bat: drag Z'
        marker.scale = 0.4
        marker.pose.position.x = self.fixed_x
        marker.pose.position.y = self.initial_y
        marker.pose.position.z = self.initial_z
        marker.pose.orientation.w = 1.0
        self.initial_pose = marker.pose
        self.marker_name = marker.name

        visual = Marker()
        visual.type = Marker.CYLINDER
        visual.scale.x = 2.0 * self.radius
        visual.scale.y = 2.0 * self.radius
        visual.scale.z = self.length
        visual.color.r = 1.0
        visual.color.g = 0.45
        visual.color.b = 0.05
        visual.color.a = 0.85
        # Rotate the cylinder's local z axis onto world x.
        visual.pose.orientation.y = math.sin(math.pi / 4.0)
        visual.pose.orientation.w = math.cos(math.pi / 4.0)

        # Z is the only draggable axis; X/Y are fixed by the feedback callback.
        handle_z = InteractiveMarkerControl()
        handle_z.always_visible = True
        handle_z.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        # RViz's MOVE_AXIS uses the matching quaternion component to select the
        # world axis; y=1 is the Z handle for this setup.
        handle_z.orientation.w = 1.0
        handle_z.orientation.y = 1.0
        handle_z.name = 'move_z'
        handle_z.markers.append(visual)
        marker.controls.append(handle_z)

        self.server.insert(marker, feedback_callback=self.feedback)
        self.server.applyChanges()
        self.publish_pose(marker.pose, self.get_clock().now().to_msg())
        if self.auto_motion:
            self.start_time = time.monotonic()
            self.create_timer(0.02, self.auto_tick)
        self.get_logger().info(
            f'bat marker ready: frame={self.frame_id} x={self.fixed_x} '
            f'length={self.length} radius={self.radius}')

    def auto_tick(self):
        elapsed = time.monotonic() - self.start_time
        pose = self.initial_pose
        pose.position.z = self.initial_z + self.motion_amplitude * math.sin(
            2.0 * math.pi * self.motion_frequency * elapsed)
        self.server.setPose(self.marker_name, pose)
        self.server.applyChanges()
        self.publish_pose(pose, self.get_clock().now().to_msg())

    def publish_pose(self, pose, stamp):
        message = PoseStamped()
        message.header.stamp = stamp
        message.header.frame_id = self.frame_id
        message.pose = pose
        self.pose_pub.publish(message)

    def feedback(self, feedback):
        if self.auto_motion:
            return
        pose = feedback.pose
        pose.position.x = self.fixed_x  # only Z is editable
        pose.position.y = self.fixed_y
        self.server.setPose(feedback.marker_name, pose)
        self.server.applyChanges()
        self.publish_pose(pose, self.get_clock().now().to_msg())


def main():
    rclpy.init()
    node = BatMarker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
