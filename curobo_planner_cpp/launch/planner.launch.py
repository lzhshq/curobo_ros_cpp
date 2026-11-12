import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    config = LaunchConfiguration('config').perform(context)
    parameters = [config]

    overrides = {}
    for name in ('artifact_dir', 'artifact_root', 'robot', 'tool_frame', 'base_frame'):
        value = LaunchConfiguration(name).perform(context)
        if value:
            overrides[name] = value
    if overrides:
        parameters.append(overrides)

    return [
        Node(
            package='curobo_planner_cpp',
            executable='planner_node',
            name='curobo_planner',
            output='screen',
            parameters=parameters,
        ),
    ]


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('curobo_planner_cpp'),
        'config',
        'planner.yaml',
    )
    default_artifact_root = os.environ.get('CUROBO_ARTIFACT_ROOT', '')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config', default_value=default_config,
            description='Parameter YAML; defaults to the installed config/planner.yaml'),
        DeclareLaunchArgument(
            'artifact_dir', default_value='',
            description='Override artifact_dir from the YAML'),
        DeclareLaunchArgument(
            'artifact_root', default_value=default_artifact_root,
            description='Artifact root; defaults to $CUROBO_ARTIFACT_ROOT'),
        DeclareLaunchArgument(
            'robot', default_value='',
            description='Override robot from the YAML'),
        DeclareLaunchArgument(
            'tool_frame', default_value='',
            description='Override tool_frame from the YAML'),
        DeclareLaunchArgument(
            'base_frame', default_value='',
            description='Override base_frame from the YAML'),
        OpaqueFunction(function=_launch_setup),
    ])
