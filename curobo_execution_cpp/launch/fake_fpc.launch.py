import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    default_joints = [
        'panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4',
        'panda_joint5', 'panda_joint6', 'panda_joint7',
    ]
    default_home = [0.0, -1.3, 0.0, -2.5, 0.0, 1.5, 0.8]
    return LaunchDescription([
        Node(
            package='curobo_execution_cpp',
            executable='fake_fpc',
            name='fake_fpc',
            output='screen',
            parameters=[{
                'joint_names': default_joints,
                'initial_positions': default_home,
                'command_topic': '/position_forward_controller/commands',
                'publish_rate': 100.0,
            }],
        ),
    ])
