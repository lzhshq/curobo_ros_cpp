"""Near-target MPC tracking of a YZ circle."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mpc_share = get_package_share_directory('curobo_mpc_cpp')
    panda_share = get_package_share_directory('moveit_resources_panda_description')
    artifact_dir = LaunchConfiguration('artifact_dir')
    robot_description = LaunchConfiguration('robot_description')
    default_artifact = os.environ.get(
        'CUROBO_ARTIFACT_DIR',
        '/home/vitai/curobo-fast/cpp_runtime/robot/artifacts/franka')

    return LaunchDescription([
        DeclareLaunchArgument('artifact_dir', default_value=default_artifact),
        DeclareLaunchArgument(
            'robot_description',
            default_value=os.path.join(panda_share, 'urdf', 'panda.urdf')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                mpc_share, 'launch', 'franka_mpc_closed_loop.launch.py')),
            launch_arguments={
                'artifact_dir': artifact_dir,
                'robot_description': robot_description,
                'autostart_cycle': 'false',
            }.items()),
        TimerAction(period=3.0, actions=[Node(
            package='curobo_mpc_cpp', executable='circle_target',
            name='circle_target', output='screen',
            parameters=[{
                'frame_id': 'panda_link0',
                'center_x': 0.161283,
                'center_y': 0.0,
                'center_z': 0.637,
                'radius': 0.04,
                'period': 5.0,
                'rate': 10.0,
            }])]),
    ])
