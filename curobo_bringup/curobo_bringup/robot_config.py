# SPDX-FileCopyrightText: Copyright (c) 2026 lzhshq
# SPDX-License-Identifier: Apache-2.0
"""Load and validate one robot descriptor.

A descriptor is the single source of truth for how a robot is wired into the
planner, executor, and RViz.  It is intentionally small: it describes names and
resources, not planner/executor tuning.
"""

from __future__ import annotations

import json
import os
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml
from ament_index_python.packages import get_package_share_directory


@dataclass(frozen=True)
class RobotConfig:
    """Validated robot descriptor."""

    name: str
    descriptor_path: Path
    artifact_dir: Path
    base_frame: str
    tool_frame: str
    artifact_joints: tuple[str, ...]
    controller_joints: tuple[str, ...]
    urdf_joints: tuple[str, ...]
    execution: dict[str, Any]
    urdf: Path
    fake_launch: Path | None

    @property
    def controller_to_artifact(self) -> str:
        """Mapping for planner start-state lookup."""
        return ",".join(
            f"{controller}:{artifact}"
            for controller, artifact in zip(
                self.controller_joints, self.artifact_joints, strict=True
            )
        )

    @property
    def artifact_to_controller(self) -> str:
        """Mapping for executor goals and ghost replay."""
        return ",".join(
            f"{artifact}:{controller}"
            for artifact, controller in zip(
                self.artifact_joints, self.controller_joints, strict=True
            )
        )

    @property
    def artifact_to_urdf(self) -> str:
        """Mapping for ghost replay into the URDF joint names."""
        return ",".join(
            f"{artifact}:{urdf}"
            for artifact, urdf in zip(self.artifact_joints, self.urdf_joints, strict=True)
        )


def _share_dir(package: str) -> Path:
    try:
        return Path(get_package_share_directory(package))
    except Exception as exception:  # ament_index_python raises KeyError
        raise ValueError(f"package '{package}' is not installed") from exception


def _resolve_package_uri(uri: str) -> Path:
    if not uri.startswith("package://"):
        return Path(uri).expanduser()
    package, _, relative = uri[len("package://") :].partition("/")
    if not package or not relative:
        raise ValueError(f"invalid package URI: {uri}")
    return _share_dir(package) / relative


def _resolve_file(uri: str, descriptor_path: Path) -> Path:
    path = _resolve_package_uri(uri)
    if path.is_absolute() and path.exists():
        return path
    candidates = [
        descriptor_path.parent / path,
        descriptor_path.parent.parent / path,
        descriptor_path.parent.parent.parent / path,
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate.resolve()
    raise FileNotFoundError(f"file not found: {uri} (from {descriptor_path})")


def _config_root() -> Path:
    configured = os.environ.get("CUROBO_ROBOT_CONFIG_DIR")
    if configured:
        return Path(configured).expanduser()
    try:
        return _share_dir("curobo_bringup") / "config" / "robots"
    except ValueError:
        return Path(__file__).resolve().parents[1] / "config" / "robots"


def _as_name_list(value: Any, field: str) -> tuple[str, ...]:
    if not isinstance(value, list) or not value:
        raise ValueError(f"{field} must be a non-empty list")
    names = tuple(str(item).strip() for item in value)
    if any(not name for name in names):
        raise ValueError(f"{field} contains an empty joint name")
    if len(set(names)) != len(names):
        raise ValueError(f"{field} contains duplicate joint names")
    return names


def _read_manifest(artifact_dir: Path) -> dict[str, Any]:
    manifest = artifact_dir / "robot_kinematics_manifest.json"
    if not manifest.is_file():
        raise FileNotFoundError(f"artifact manifest not found: {manifest}")
    with manifest.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def _read_urdf_joints(urdf: Path) -> set[str]:
    root = ET.parse(urdf).getroot()
    return {
        joint.get("name", "")
        for joint in root.findall("joint")
        if joint.get("name")
    }


def load_robot_config(
    robot: str,
    artifact_root: str | os.PathLike[str] | None = None,
) -> RobotConfig:
    """Load ``robot`` (name or YAML path) and validate all resources.

    Raises:
        FileNotFoundError: a descriptor, artifact, URDF, or fake launch is missing.
        ValueError: joint lists, manifest, or execution parameters disagree.
    """
    descriptor = Path(robot).expanduser()
    if not descriptor.is_file():
        descriptor = _config_root() / f"{robot}.yaml"
    if not descriptor.is_file():
        raise FileNotFoundError(f"robot descriptor not found: {robot}")

    with descriptor.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict):
        raise ValueError(f"descriptor must be a mapping: {descriptor}")

    name = str(data.get("name", descriptor.stem))
    base_frame = str(data.get("base_frame", "")).strip()
    tool_frame = str(data.get("tool_frame", "")).strip()
    if not base_frame:
        raise ValueError(f"{descriptor}: base_frame is required")

    joints = data.get("joints")
    if not isinstance(joints, dict):
        raise ValueError(f"{descriptor}: joints mapping is required")
    artifact_joints = _as_name_list(joints.get("artifact"), "joints.artifact")
    controller_joints = _as_name_list(joints.get("controller"), "joints.controller")
    urdf_joints = _as_name_list(joints.get("urdf"), "joints.urdf")
    if not (len(artifact_joints) == len(controller_joints) == len(urdf_joints)):
        raise ValueError(
            f"{descriptor}: joint list lengths differ: "
            f"artifact={len(artifact_joints)}, controller={len(controller_joints)}, "
            f"urdf={len(urdf_joints)}"
        )

    artifact_setting = str(data.get("artifact", name)).strip()
    if not artifact_setting:
        raise ValueError(f"{descriptor}: artifact is required")
    artifact_dir = Path(artifact_setting).expanduser()
    if not artifact_dir.is_absolute():
        root = artifact_root or os.environ.get("CUROBO_ARTIFACT_ROOT")
        if not root:
            raise ValueError(
                f"{descriptor}: set CUROBO_ARTIFACT_ROOT or provide artifact_dir"
            )
        artifact_dir = Path(root).expanduser() / artifact_setting
    artifact_dir = artifact_dir.resolve()

    manifest = _read_manifest(artifact_dir)
    manifest_joints = tuple(str(item) for item in manifest.get("joint_names", []))
    if manifest_joints != artifact_joints:
        raise ValueError(
            f"{descriptor}: artifact joints {list(artifact_joints)} do not match "
            f"manifest joints {list(manifest_joints)}"
        )
    if manifest.get("base_link") != base_frame:
        raise ValueError(
            f"{descriptor}: base_frame '{base_frame}' does not match artifact "
            f"base_link '{manifest.get('base_link')}'"
        )
    tool_frames = tuple(str(item) for item in manifest.get("tool_frames", []))
    if tool_frame and tool_frame not in tool_frames:
        raise ValueError(
            f"{descriptor}: tool_frame '{tool_frame}' is not in artifact tool frames "
            f"{list(tool_frames)}"
        )
    if not tool_frame and tool_frames:
        tool_frame = tool_frames[0]

    visualization = data.get("visualization")
    if not isinstance(visualization, dict) or not visualization.get("urdf"):
        raise ValueError(f"{descriptor}: visualization.urdf is required")
    urdf = _resolve_file(str(visualization["urdf"]), descriptor)
    urdf_joint_names = _read_urdf_joints(urdf)
    missing = [joint for joint in urdf_joints if joint not in urdf_joint_names]
    if missing:
        raise ValueError(
            f"{descriptor}: URDF {urdf} is missing joints {missing}"
        )

    execution = data.get("execution")
    if not isinstance(execution, dict):
        raise ValueError(f"{descriptor}: execution mapping is required")
    adapter = str(execution.get("adapter", "jtc")).strip()
    action = str(execution.get("action", "")).strip()
    if not adapter:
        raise ValueError(f"{descriptor}: execution.adapter is required")
    if not action:
        raise ValueError(f"{descriptor}: execution.action is required")
    execution_params: dict[str, Any] = {
        "adapter": adapter,
        "action": action,
        "joint_names": list(controller_joints),
    }
    for key in ("time_scale", "goal_tolerance", "goal_time_tolerance"):
        if key in execution:
            execution_params[key] = execution[key]

    fake_launch: Path | None = None
    fake_setting = str(data.get("fake_launch", "")).strip()
    if fake_setting:
        fake_launch = _resolve_file(fake_setting, descriptor)

    return RobotConfig(
        name=name,
        descriptor_path=descriptor.resolve(),
        artifact_dir=artifact_dir,
        base_frame=base_frame,
        tool_frame=tool_frame,
        artifact_joints=artifact_joints,
        controller_joints=controller_joints,
        urdf_joints=urdf_joints,
        execution=execution_params,
        urdf=urdf,
        fake_launch=fake_launch,
    )
