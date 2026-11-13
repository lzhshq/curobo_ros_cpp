import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    mpc_share = get_package_share_directory('curobo_mpc_cpp')
    artifact = LaunchConfiguration('artifact_dir')
    default = os.environ.get('CUROBO_ARTIFACT_DIR',
        '/home/vitai/curobo-fast/cpp_runtime/robot/artifacts/franka')
    return LaunchDescription([
        DeclareLaunchArgument('artifact_dir', default_value=default),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                mpc_share, 'launch', 'franka_mpc_closed_loop.launch.py')),
            launch_arguments={'artifact_dir': artifact,
                              'autostart_cycle': 'false'}.items()),
        Node(package='curobo_unified_planner_cpp', executable='unified_planner',
             name='unified_planner', output='screen'),
    ])
