from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('base_frame', default_value='panda_link0'),
        DeclareLaunchArgument('tool_frame', default_value='panda_hand'),
        DeclareLaunchArgument('goal_topic', default_value='/curobo_mpc/goal'),
        DeclareLaunchArgument('position_tolerance', default_value='0.01'),
        DeclareLaunchArgument('orientation_tolerance', default_value='0.20'),
        DeclareLaunchArgument('dwell', default_value='0.6'),
        DeclareLaunchArgument('max_hold', default_value='5.0'),
        Node(
            package='curobo_mpc_cpp',
            executable='target_cycle',
            name='target_cycle',
            output='screen',
            parameters=[{
                'base_frame': LaunchConfiguration('base_frame'),
                'tool_frame': LaunchConfiguration('tool_frame'),
                'goal_topic': LaunchConfiguration('goal_topic'),
                'position_tolerance': LaunchConfiguration('position_tolerance'),
                'orientation_tolerance': LaunchConfiguration('orientation_tolerance'),
                'dwell': LaunchConfiguration('dwell'),
                'max_hold': LaunchConfiguration('max_hold'),
            }],
        ),
    ])
