import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _make_rviz(context, *args, **kwargs):
    config = LaunchConfiguration('rviz_config').perform(context)
    base_frame = LaunchConfiguration('base_frame').perform(context)
    robot_description = LaunchConfiguration('robot_description').perform(context)
    show_robot = LaunchConfiguration('show_robot').perform(context).lower() in (
        '1', 'true', 'yes')
    show_ghost = LaunchConfiguration('show_ghost').perform(context).lower() in (
        '1', 'true', 'yes')
    ghost_time_scale = float(LaunchConfiguration('ghost_time_scale').perform(context))
    ghost_loop = LaunchConfiguration('ghost_loop').perform(context).lower() in (
        '1', 'true', 'yes')
    ghost_joint_name_map = LaunchConfiguration('ghost_joint_name_map').perform(context)

    try:
        with open(config, 'r') as handle:
            data = yaml.safe_load(handle)
        manager = data.setdefault('Visualization Manager', {})
        options = manager.setdefault('Global Options', {})
        options['Fixed Frame'] = base_frame
        for display in manager.get('Displays', []):
            if display.get('Class') != 'rviz_default_plugins/RobotModel':
                continue
            if display.get('Name') == 'RobotModel':
                display['Enabled'] = show_robot
                display['Value'] = show_robot
            elif display.get('Name') == 'GhostRobot':
                display['Enabled'] = show_ghost and bool(robot_description)
                display['Value'] = show_ghost and bool(robot_description)
        temp = tempfile.NamedTemporaryFile(mode='w', suffix='.rviz', delete=False)
        yaml.safe_dump(data, temp, default_flow_style=False)
        temp.close()
        config = temp.name
    except OSError as exception:
        print(f'[rviz] could not patch config: {exception}; using original')

    nodes = []
    urdf = ''
    if robot_description:
        try:
            with open(robot_description, 'r') as handle:
                urdf = handle.read()
        except OSError as exception:
            print(f'[rviz] could not read robot_description: {exception}')
    elif show_robot or show_ghost:
        print('[rviz] show_robot/show_ghost is true but robot_description is empty')

    if urdf and show_robot:
        nodes.append(Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': urdf}],
        ))

    if urdf and show_ghost:
        nodes.append(Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace='preview',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': urdf,
                'frame_prefix': 'preview/',
            }],
        ))
        nodes.append(Node(
            package='curobo_planner_cpp',
            executable='preview_replay',
            name='preview_replay',
            output='screen',
            parameters=[{
                'trajectory_topic': '/curobo_planner/planned_trajectory',
                'joint_states_topic': '/preview/joint_states',
                'time_scale': ghost_time_scale,
                'loop': ghost_loop,
                'joint_name_map': ghost_joint_name_map,
            }],
        ))
        nodes.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='preview_base_tf',
            arguments=['0', '0', '0', '0', '0', '0',
                       base_frame, 'preview/' + base_frame],
        ))

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
            'rviz_config', default_value=default_config,
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
        DeclareLaunchArgument(
            'show_ghost', default_value='false',
            description='Enable translucent ghost trajectory preview'),
        DeclareLaunchArgument(
            'ghost_time_scale', default_value='1.0',
            description='Ghost replay speed multiplier'),
        DeclareLaunchArgument(
            'ghost_loop', default_value='true',
            description='Loop the ghost replay from red to green'),
        DeclareLaunchArgument(
            'ghost_joint_name_map', default_value='',
            description='Artifact:URDF joint names for the ghost, e.g. j1:joint1,...'),
        OpaqueFunction(function=_make_rviz),
    ])
