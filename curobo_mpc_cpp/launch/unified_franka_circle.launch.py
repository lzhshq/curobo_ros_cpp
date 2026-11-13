import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    share = get_package_share_directory('curobo_mpc_cpp')
    artifact = LaunchConfiguration('artifact_dir')
    default = os.environ.get('CUROBO_ARTIFACT_DIR',
        '/home/vitai/curobo-fast/cpp_runtime/robot/artifacts/franka')
    return LaunchDescription([
        DeclareLaunchArgument('artifact_dir', default_value=default),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                share, 'launch', 'franka_mpc_closed_loop.launch.py')),
            launch_arguments={'artifact_dir': artifact,
                              'autostart_cycle': 'false'}.items()),
        TimerAction(period=4.0, actions=[Node(
            package='curobo_mpc_cpp', executable='unified_circle_target',
            name='unified_circle_target', output='screen',
            parameters=[{
                'center_x': 0.161283, 'center_y': 0.0, 'center_z': 0.637,
                'radius': 0.04, 'period': 10.0, 'rate': 10.0}])]),
    ])
