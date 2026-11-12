# SPDX-FileCopyrightText: Copyright (c) 2026 lzhshq
# SPDX-License-Identifier: Apache-2.0
"""Start planner + executor + RViz from one robot descriptor."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from curobo_bringup.robot_config import load_robot_config


def _as_bool(value: str) -> bool:
    return value.lower() in ("1", "true", "yes", "on")


def _launch_setup(context, *args, **kwargs):
    robot = LaunchConfiguration("robot").perform(context)
    artifact_root = LaunchConfiguration("artifact_root").perform(context)
    fake_setting = LaunchConfiguration("fake").perform(context).lower()
    rviz = _as_bool(LaunchConfiguration("rviz").perform(context))
    show_robot = LaunchConfiguration("show_robot").perform(context)
    show_ghost = LaunchConfiguration("show_ghost").perform(context)
    ghost_time_scale = LaunchConfiguration("ghost_time_scale").perform(context)
    ghost_loop = LaunchConfiguration("ghost_loop").perform(context)

    config = load_robot_config(robot, artifact_root=artifact_root or None)
    print(
        f"[robot_stack] {config.name}: artifact={config.artifact_dir} "
        f"base={config.base_frame} tool={config.tool_frame}"
    )
    print(f"[robot_stack] controller->artifact: {config.controller_to_artifact}")
    print(f"[robot_stack] artifact->urdf:      {config.artifact_to_urdf}")

    fake = config.fake_launch is not None if fake_setting == "auto" else _as_bool(fake_setting)
    if fake and config.fake_launch is None:
        raise RuntimeError(
            f"{config.name} has no fake_launch in its descriptor; start its hardware "
            "separately or pass fake:=false"
        )

    actions = []
    if fake:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(config.fake_launch)),
        ))

    planner_launch = os.path.join(
        get_package_share_directory("curobo_planner_cpp"),
        "launch",
        "planner.launch.py",
    )
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(planner_launch),
        launch_arguments={
            "artifact_dir": str(config.artifact_dir),
            "base_frame": config.base_frame,
            "tool_frame": config.tool_frame,
            "joint_name_map": config.controller_to_artifact,
            "robot": config.name,
        }.items(),
    ))

    execution_launch = os.path.join(
        get_package_share_directory("curobo_execution_cpp"),
        "launch",
        "execution.launch.py",
    )
    execution_arguments = {
        "adapter": str(config.execution["adapter"]),
        "action": str(config.execution["action"]),
        "joint_names": ",".join(config.controller_joints),
    }
    for key in ("time_scale", "goal_tolerance", "goal_time_tolerance"):
        if key in config.execution:
            execution_arguments[key] = str(config.execution[key])
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(execution_launch),
        launch_arguments=execution_arguments.items(),
    ))

    if rviz:
        rviz_launch = os.path.join(
            get_package_share_directory("curobo_planner_cpp"),
            "launch",
            "rviz.launch.py",
        )
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rviz_launch),
            launch_arguments={
                "base_frame": config.base_frame,
                "robot_description": str(config.urdf),
                "show_robot": show_robot,
                "show_ghost": show_ghost,
                "ghost_time_scale": ghost_time_scale,
                "ghost_loop": ghost_loop,
                "ghost_joint_name_map": config.artifact_to_urdf,
            }.items(),
            condition=IfCondition(LaunchConfiguration("rviz")),
        ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "robot", default_value="ar5",
            description="Robot descriptor name or YAML path"),
        DeclareLaunchArgument(
            "artifact_root", default_value=os.environ.get("CUROBO_ARTIFACT_ROOT", ""),
            description="Artifact root; defaults to $CUROBO_ARTIFACT_ROOT"),
        DeclareLaunchArgument(
            "fake", default_value="auto",
            description="auto/true/false; auto starts fake_launch when present"),
        DeclareLaunchArgument(
            "rviz", default_value="true",
            description="Start RViz and the ghost preview"),
        DeclareLaunchArgument(
            "show_robot", default_value="true",
            description="Enable the real RobotModel"),
        DeclareLaunchArgument(
            "show_ghost", default_value="true",
            description="Enable the translucent ghost replay"),
        DeclareLaunchArgument(
            "ghost_time_scale", default_value="0.2",
            description="Ghost replay speed multiplier"),
        DeclareLaunchArgument(
            "ghost_loop", default_value="true",
            description="Loop the ghost replay from red to green"),
        OpaqueFunction(function=_launch_setup),
    ])
