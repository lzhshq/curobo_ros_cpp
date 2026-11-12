from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='curobo_execution_cpp',
            executable='fake_jtc',
            name='fake_jtc',
            output='screen',
            parameters=[{
                'joint_names': [
                    'panda_joint1', 'panda_joint2', 'panda_joint3',
                    'panda_joint4', 'panda_joint5', 'panda_joint6',
                    'panda_joint7',
                ],
                'action': '/position_joint_trajectory_controller/follow_joint_trajectory',
                'publish_rate': 100.0,
                'initial_positions': [
                    0.0, -1.3, 0.0, -2.5, 0.0, 1.5, 0.8,
                ],
            }],
        ),
    ])
