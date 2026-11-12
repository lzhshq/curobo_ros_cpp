# SPDX-FileCopyrightText: Copyright (c) 2026 lzhshq
# SPDX-License-Identifier: Apache-2.0
import os

import pytest

from curobo_bringup.robot_config import load_robot_config


def _require_artifacts() -> None:
    if not os.environ.get("CUROBO_ARTIFACT_ROOT"):
        pytest.skip("set CUROBO_ARTIFACT_ROOT to run descriptor tests")


def test_ar5_descriptor_builds_mappings() -> None:
    _require_artifacts()
    config = load_robot_config("ar5")
    assert config.base_frame == "right_arm_base"
    assert config.tool_frame == "flange"
    assert config.artifact_joints == ("j1", "j2", "j3", "j4", "j5", "j6", "j7")
    assert config.controller_joints == (
        "joint1", "joint2", "joint3", "joint4",
        "joint5", "joint6", "joint7",
    )
    assert config.controller_to_artifact.startswith("joint1:j1,")
    assert config.artifact_to_urdf.startswith("j1:joint1,")
    assert config.urdf.is_file()
    assert config.fake_launch is None


def test_franka_fpc_descriptor() -> None:
    _require_artifacts()
    config = load_robot_config("franka_fpc")
    assert config.execution["adapter"] == "fpc"
    assert config.execution["command_topic"] == "/position_forward_controller/commands"
    assert config.execution["publish_rate"] == 100.0
    assert config.fake_launch is not None


def test_franka_descriptor_builds_mappings() -> None:
    _require_artifacts()
    config = load_robot_config("franka")
    assert config.base_frame == "panda_link0"
    assert config.tool_frame == "panda_hand"
    assert config.controller_to_artifact.startswith("panda_joint1:panda_joint1,")
    assert config.artifact_to_controller == config.controller_to_artifact
    assert config.urdf.is_file()
    assert config.fake_launch is not None
