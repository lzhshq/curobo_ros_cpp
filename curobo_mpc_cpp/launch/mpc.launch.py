import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    parameters = [LaunchConfiguration('mpc_config').perform(context)]
    artifact_dir = LaunchConfiguration('artifact_dir').perform(context)
    if artifact_dir:
        parameters.append({'artifact_dir': artifact_dir})
    command_topic = LaunchConfiguration('command_topic').perform(context)
    if command_topic:
        parameters.append({'command_topic': command_topic})
    return [
        Node(
            package='curobo_mpc_cpp',
            executable='mpc_node',
            name='curobo_mpc',
            output='screen',
            parameters=parameters,
        ),
    ]


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('curobo_mpc_cpp'), 'config', 'mpc.yaml')
    return LaunchDescription([
        DeclareLaunchArgument(
            'mpc_config', default_value=default_config,
            description='MPC parameter YAML'),
        DeclareLaunchArgument(
            'artifact_dir', default_value='',
            description='Artifact directory override'),
        DeclareLaunchArgument(
            'command_topic', default_value='',
            description='FPC command topic override'),
        OpaqueFunction(function=_launch_setup),
    ])
