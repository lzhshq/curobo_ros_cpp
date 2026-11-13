import os
import tempfile

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    parameters = [LaunchConfiguration('executor_config').perform(context)]

    overrides = {}
    for name in ('adapter', 'action', 'command_topic'):
        value = LaunchConfiguration(name).perform(context)
        if value:
            overrides[name] = value

    joint_names = LaunchConfiguration('joint_names').perform(context)
    if joint_names:
        overrides['joint_names'] = [
            name.strip() for name in joint_names.split(',') if name.strip()
        ]

    for name in ('time_scale', 'goal_tolerance', 'goal_time_tolerance', 'publish_rate'):
        value = LaunchConfiguration(name).perform(context)
        if value:
            overrides[name] = float(value)

    if overrides:
        temp = tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False)
        yaml.safe_dump(
            {'curobo_executor': {'ros__parameters': overrides}},
            temp,
            default_flow_style=False,
        )
        temp.close()
        parameters.append(temp.name)

    return [
        Node(
            package='curobo_execution_cpp',
            executable='trajectory_executor',
            name='curobo_executor',
            output='screen',
            parameters=parameters,
        ),
    ]


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('curobo_execution_cpp'),
        'config',
        'executor.yaml',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'executor_config', default_value=default_config,
            description='Executor parameter YAML'),
        DeclareLaunchArgument(
            'adapter', default_value='',
            description='Override adapter (jtc/topic)'),
        DeclareLaunchArgument(
            'action', default_value='',
            description='Override JTC action name'),
        DeclareLaunchArgument(
            'command_topic', default_value='',
            description='Override topic command'),
        DeclareLaunchArgument(
            'joint_names', default_value='',
            description='Override controller joint names, comma-separated'),
        DeclareLaunchArgument(
            'time_scale', default_value='',
            description='Override time_scale'),
        DeclareLaunchArgument(
            'goal_tolerance', default_value='',
            description='Override goal_tolerance'),
        DeclareLaunchArgument(
            'goal_time_tolerance', default_value='',
            description='Override goal_time_tolerance'),
        DeclareLaunchArgument(
            'publish_rate', default_value='',
            description='Override stream publish rate'),
        OpaqueFunction(function=_launch_setup),
    ])
