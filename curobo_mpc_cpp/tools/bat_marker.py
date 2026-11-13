#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 lzhshq
# SPDX-License-Identifier: Apache-2.0
"""RViz interactive marker for a bat-like obstacle.

The marker moves only in the world Y/Z plane; world X is fixed. The visual
cylinder's long axis is world X.
"""

import math

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
        self.fixed_y = float(self.declare_parameter('y', 0.0).value)
        self.initial_z = float(self.declare_parameter('z', 0.30).value)
        self.length = float(self.declare_parameter('length', 0.6).value)
        self.radius = float(self.declare_parameter('radius', 0.03).value)
        topic = self.declare_parameter(
            'pose_topic', '/curobo_demo/bat_pose').value

        self.pose_pub = self.create_publisher(PoseStamped, topic, 10)
        self.server = InteractiveMarkerServer(self, 'curobo_demo/bat')

        marker = InteractiveMarker()
        marker.header.frame_id = self.frame_id
        marker.name = 'bat'
        marker.description = 'bat: drag Y/Z'
        marker.scale = 0.4
        marker.pose.position.x = self.fixed_x
        marker.pose.position.y = self.fixed_y
        marker.pose.position.z = self.initial_z
        marker.pose.orientation.w = 1.0

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

        # Only Z is draggable. X and Y are fixed by the feedback callback.
        handle_z = InteractiveMarkerControl()
        handle_z.always_visible = True
        handle_z.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        handle_z.orientation.w = 1.0
        handle_z.name = 'move_z'
        handle_z.markers.append(visual)
        marker.controls.append(handle_z)

        self.server.insert(marker, feedback_callback=self.feedback)
        self.server.applyChanges()
        self.publish_pose(marker.pose, self.get_clock().now().to_msg())
        self.get_logger().info(
            f'bat marker ready: frame={self.frame_id} x={self.fixed_x} '
            f'length={self.length} radius={self.radius}')

    def publish_pose(self, pose, stamp):
        message = PoseStamped()
        message.header.stamp = stamp
        message.header.frame_id = self.frame_id
        message.pose = pose
        self.pose_pub.publish(message)

    def feedback(self, feedback):
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
