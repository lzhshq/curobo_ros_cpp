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
    robot_description = LaunchConfiguration('robot_description').perform(context)
    show_robot = LaunchConfiguration('show_robot').perform(context).lower() in (
        '1', 'true', 'yes')

    try:
        with open(config, 'r') as handle:
            data = yaml.safe_load(handle)
        manager = data.setdefault('Visualization Manager', {})
        options = manager.setdefault('Global Options', {})
        options['Fixed Frame'] = base_frame
        for display in manager.get('Displays', []):
            if display.get('Class') == 'rviz_default_plugins/RobotModel':
                display['Enabled'] = show_robot
                display['Value'] = show_robot
        temp = tempfile.NamedTemporaryFile(mode='w', suffix='.rviz', delete=False)
        yaml.safe_dump(data, temp, default_flow_style=False)
        temp.close()
        config = temp.name
    except OSError as exception:
        print(f'[rviz] could not patch config: {exception}; using original')

    nodes = []
    if robot_description:
        try:
            with open(robot_description, 'r') as handle:
                urdf = handle.read()
            nodes.append(Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{'robot_description': urdf}],
            ))
        except OSError as exception:
            print(f'[rviz] could not read robot_description: {exception}')
    elif show_robot:
        print('[rviz] show_robot is true but robot_description is empty')

    nodes.append(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', config],
    ))
    return nodes


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
        DeclareLaunchArgument(
            'robot_description', default_value='',
            description='URDF file for robot_state_publisher'),
        DeclareLaunchArgument(
            'show_robot', default_value='false',
            description='Enable RViz RobotModel'),
        OpaqueFunction(function=_make_rviz),
    ])
