"""Near-target MPC closed-loop demo.

MPC sends a short 4-step segment (about 0.12 s), waits for execution, then
re-solves from measured joint state. No global planner is used for these goals.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mpc_share = get_package_share_directory('curobo_mpc_cpp')
    execution_share = get_package_share_directory('curobo_execution_cpp')
    planner_share = get_package_share_directory('curobo_planner_cpp')
    panda_share = get_package_share_directory('moveit_resources_panda_description')

    artifact_dir = LaunchConfiguration('artifact_dir')
    robot_description = LaunchConfiguration('robot_description')
    default_artifact = os.environ.get(
        'CUROBO_ARTIFACT_DIR',
        '/home/vitai/curobo-fast/cpp_runtime/robot/artifacts/franka')
    q = [-0.988745, 0.007219, -0.149434, 0.001091]
    target_a = [0.161283, 0.04, 0.645, *q]
    target_b = [0.161283, -0.04, 0.645, *q]

    return LaunchDescription([
        DeclareLaunchArgument('artifact_dir', default_value=default_artifact),
        DeclareLaunchArgument('autostart_cycle', default_value='true'),
        DeclareLaunchArgument('brake_check', default_value='true'),
        DeclareLaunchArgument('control_rate', default_value='30.0'),
        DeclareLaunchArgument(
            'robot_description',
            default_value=os.path.join(panda_share, 'urdf', 'panda.urdf')),
        Node(
            package='curobo_execution_cpp', executable='fake_jtc', name='fake_jtc',
            output='screen',
            parameters=[{
                'joint_names': [f'panda_joint{i}' for i in range(1, 8)],
                'action': '/position_joint_trajectory_controller/follow_joint_trajectory',
                'publish_rate': 100.0,
                'initial_positions': [0.0, -1.3, 0.0, -2.5, 0.0, 1.5, 0.8],
            }]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                execution_share, 'launch', 'execution.launch.py'))),
        Node(
            package='curobo_mpc_cpp', executable='mpc_node', name='curobo_mpc',
            output='screen',
            parameters=[
                os.path.join(mpc_share, 'config', 'mpc.yaml'),
                {
                    'artifact_dir': artifact_dir,
                    'adapter': 'jtc',
                    'planner_distance_threshold': 100.0,
                    'command_steps': 16,
                    'use_measured_state': True,
                    'adapter': 'segment',
                    'brake_check': LaunchConfiguration('brake_check'),
                    'control_rate': LaunchConfiguration('control_rate'),
                },
            ]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                planner_share, 'launch', 'rviz.launch.py')),
            launch_arguments={
                'base_frame': 'panda_link0',
                'robot_description': robot_description,
                'show_robot': 'true',
                'show_ghost': 'false',
            }.items()),
        TimerAction(period=3.0, actions=[Node(
            package='curobo_mpc_cpp', executable='target_cycle',
            name='target_cycle', output='screen',
            condition=IfCondition(LaunchConfiguration('autostart_cycle')),
            parameters=[{
                'base_frame': 'panda_link0',
                'tool_frame': 'panda_hand',
                'target_a': target_a,
                'target_b': target_b,
                'position_tolerance': 0.01,
                'max_hold': 8.0,
            }])]),
    ])
