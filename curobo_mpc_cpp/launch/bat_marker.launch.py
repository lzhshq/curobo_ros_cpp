import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('frame_id', default_value='panda_link0'),
        DeclareLaunchArgument('x', default_value='0.30'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.30'),
        DeclareLaunchArgument('length', default_value='0.4'),
        DeclareLaunchArgument('radius', default_value='0.03'),
        DeclareLaunchArgument(
            'pose_topic', default_value='/curobo_demo/bat_pose'),
        Node(
            package='curobo_mpc_cpp',
            executable='bat_marker',
            name='bat_marker',
            output='screen',
            parameters=[{
                'frame_id': LaunchConfiguration('frame_id'),
                'x': LaunchConfiguration('x'),
                'y': LaunchConfiguration('y'),
                'z': LaunchConfiguration('z'),
                'length': LaunchConfiguration('length'),
                'radius': LaunchConfiguration('radius'),
                'pose_topic': LaunchConfiguration('pose_topic'),
            }],
        ),
    ])
