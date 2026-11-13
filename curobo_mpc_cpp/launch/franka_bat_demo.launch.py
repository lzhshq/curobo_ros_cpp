"""One-command Franka bat/MPC demo.

Usage:
    ros2 launch curobo_mpc_cpp franka_bat_demo.launch.py

Optional overrides (defaults are the installed demo):
    artifact_dir:=... robot_description:=...
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    mpc_share = get_package_share_directory('curobo_mpc_cpp')
    execution_share = get_package_share_directory('curobo_execution_cpp')
    planner_share = get_package_share_directory('curobo_planner_cpp')
    panda_share = get_package_share_directory('moveit_resources_panda_description')

    artifact_dir = LaunchConfiguration('artifact_dir')
    robot_description = LaunchConfiguration('robot_description')

    return LaunchDescription([
        DeclareLaunchArgument(
            'artifact_dir',
            default_value='/home/vitai/curobo-fast/cpp_runtime/robot/artifacts/franka',
            description='cuRobo Fast artifact directory'),
        DeclareLaunchArgument(
            'robot_description',
            default_value=os.path.join(panda_share, 'urdf', 'panda.urdf'),
            description='URDF used by robot_state_publisher/RViz'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                execution_share, 'launch', 'fake_franka.launch.py'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                mpc_share, 'launch', 'mpc.launch.py')),
            launch_arguments={'artifact_dir': artifact_dir}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                planner_share, 'launch', 'rviz.launch.py')),
            launch_arguments={
                'base_frame': 'panda_link0',
                'robot_description': robot_description,
                'show_robot': 'true',
                'show_ghost': 'false',
            }.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                mpc_share, 'launch', 'bat_marker.launch.py'))),
        TimerAction(
            period=3.0,
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    mpc_share, 'launch', 'target_cycle.launch.py')))]),
    ])
