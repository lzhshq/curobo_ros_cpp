from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'artifact_root', default_value='',
            description='Directory containing <robot>/ artifact directories'),
        DeclareLaunchArgument(
            'robot', default_value='',
            description='Artifact directory name under artifact_root'),
        DeclareLaunchArgument(
            'artifact_dir', default_value='',
            description='Absolute artifact directory; overrides artifact_root/robot'),
        DeclareLaunchArgument(
            'tool_frame', default_value='',
            description='Tool frame from the artifact; empty uses the first frame'),
        DeclareLaunchArgument(
            'base_frame', default_value='',
            description='Frame id written to the published JointTrajectory'),

        Node(
            package='curobo_planner_cpp',
            executable='planner_node',
            name='curobo_planner',
            output='screen',
            parameters=[{
                'artifact_root': LaunchConfiguration('artifact_root'),
                'robot': LaunchConfiguration('robot'),
                'artifact_dir': LaunchConfiguration('artifact_dir'),
                'tool_frame': LaunchConfiguration('tool_frame'),
                'base_frame': LaunchConfiguration('base_frame'),
            }],
        ),
    ])
