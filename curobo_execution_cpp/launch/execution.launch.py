import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('curobo_execution_cpp'),
        'config',
        'executor.yaml',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'config', default_value=default_config,
            description='Executor parameter YAML'),
        Node(
            package='curobo_execution_cpp',
            executable='trajectory_executor',
            name='curobo_executor',
            output='screen',
            parameters=[LaunchConfiguration('config')],
        ),
    ])
