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
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from rclpy.node import Node
from curobo_planner_interfaces.msg import ObstacleUpdate
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
        self.world_topic = self.declare_parameter(
            'world_topic', '/curobo_world/obstacle_update').value

        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE)
        self.world_pub = self.create_publisher(ObstacleUpdate, self.world_topic, qos)
        self.server = InteractiveMarkerServer(self, 'curobo_demo/bat')

        marker = InteractiveMarker()
        marker.header.frame_id = self.frame_id
        marker.name = 'bat'
        marker.description = 'bat: automatic Z'
        marker.scale = 0.4
        marker.pose.position.x = self.fixed_x
        marker.pose.position.y = self.initial_y
        marker.pose.position.z = self.initial_z
        marker.pose.orientation.w = 1.0
        self.initial_pose = marker.pose
        self.marker_name = marker.name

        visual = Marker()
        # The control frame maps the visual's local +Z onto world +X, so the
        # cylinder's length goes on scale.z.
        visual.type = Marker.CYLINDER
        visual.scale.x = 2.0 * self.radius
        visual.scale.y = 2.0 * self.radius
        visual.scale.z = self.length
        visual.color.r = 1.0
        visual.color.g = 0.45
        visual.color.b = 0.05
        visual.color.a = 0.85
        visual.pose.orientation.w = 1.0
        # Demo bat is automatic only: render the visual in an identity control
        # and expose no drag handles.
        visual_control = InteractiveMarkerControl()
        visual_control.always_visible = True
        visual_control.interaction_mode = InteractiveMarkerControl.NONE
        visual_control.orientation.w = 1.0
        visual_control.name = 'bat_visual'
        visual_control.markers.append(visual)
        marker.controls.append(visual_control)

        self.server.insert(marker)
        self.server.applyChanges()
        self.publish_update(ObstacleUpdate.ADD, marker.pose)
        if self.auto_motion:
            self.start_time = time.monotonic()
            self.create_timer(0.05, self.auto_tick)
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
        self.publish_update(ObstacleUpdate.UPDATE, pose)

    def publish_update(self, command, pose):
        message = ObstacleUpdate()
        message.command = command
        message.name = 'bat'
        message.type = 2  # CAPSULE
        message.pose = pose
        # The capsule's local axis is +Z; rotate it onto world +X.
        message.pose.orientation.x = 0.0
        message.pose.orientation.y = math.sin(math.pi / 4.0)
        message.pose.orientation.z = 0.0
        message.pose.orientation.w = math.cos(math.pi / 4.0)
        message.dimensions.x = self.radius
        message.dimensions.y = self.length
        message.dimensions.z = 0.0
        message.stamp = self.get_clock().now().to_msg()
        self.world_pub.publish(message)

    def feedback(self, feedback):
        if self.auto_motion:
            return
        pose = feedback.pose
        pose.position.x = self.fixed_x  # only Z is editable
        pose.position.y = self.initial_y
        self.server.setPose(feedback.marker_name, pose)
        self.server.applyChanges()
        self.publish_update(ObstacleUpdate.UPDATE, pose)


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
