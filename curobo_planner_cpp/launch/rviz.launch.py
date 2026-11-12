import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _make_rviz(context, *args, **kwargs):
    config = LaunchConfiguration('config').perform(context)
    base_frame = LaunchConfiguration('base_frame').perform(context)

    try:
        with open(config, 'r') as handle:
            data = yaml.safe_load(handle)
        manager = data.setdefault('Visualization Manager', {})
        options = manager.setdefault('Global Options', {})
        options['Fixed Frame'] = base_frame
        temp = tempfile.NamedTemporaryFile(mode='w', suffix='.rviz', delete=False)
        yaml.safe_dump(data, temp, default_flow_style=False)
        temp.close()
        config = temp.name
    except OSError as exception:
        print(f'[rviz] could not patch config: {exception}; using original')

    return [Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', config],
    )]


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('curobo_planner_cpp'), 'rviz', 'planner.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config', default_value=default_config,
            description='RViz config file'),
        DeclareLaunchArgument(
            'base_frame', default_value='right_arm_base',
            description='RViz Fixed Frame; must match the artifact base frame'),
        OpaqueFunction(function=_make_rviz),
    ])
