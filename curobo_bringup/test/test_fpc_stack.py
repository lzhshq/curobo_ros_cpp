# SPDX-FileCopyrightText: Copyright (c) 2026 lzhshq
# SPDX-License-Identifier: Apache-2.0
"""End-to-end FPC stack test: fake FPC plant -> planner -> executor."""

import os
import signal
import subprocess
import tempfile
import time

from geometry_msgs.msg import Point, Pose, Quaternion
import pytest
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from curobo_planner_interfaces.srv import PlanPose

DOMAIN_ID = "92"


def _artifact_root() -> str | None:
    root = os.environ.get("CUROBO_ARTIFACT_ROOT", "")
    manifest = os.path.join(root, "franka", "robot_kinematics_manifest.json")
    return root if root and os.path.isfile(manifest) else None


def _call(node, client, request, timeout_sec=60.0):
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
    assert future.done(), f"service call timed out: {client.srv_name}"
    return future.result()


@pytest.fixture(scope="module")
def stack():
    artifact_root = _artifact_root()
    if artifact_root is None:
        pytest.skip("set CUROBO_ARTIFACT_ROOT with a franka artifact")

    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = DOMAIN_ID
    env["CUROBO_ARTIFACT_ROOT"] = artifact_root
    os.environ["ROS_DOMAIN_ID"] = DOMAIN_ID

    log = tempfile.TemporaryFile(mode="w+")
    process = subprocess.Popen(
        [
            "ros2", "launch", "curobo_bringup", "robot_stack.launch.py",
            "robot:=franka_fpc", "fake:=auto", "rviz:=false",
        ],
        env=env,
        stdout=log,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    )

    rclpy.init()
    node = Node("test_fpc_stack")
    clients = {
        "plan": node.create_client(PlanPose, "/curobo_planner/generate_trajectory"),
        "execute": node.create_client(Trigger, "/curobo_executor/execute_latest"),
        "status": node.create_client(Trigger, "/curobo_executor/status"),
    }

    try:
        for name, client in clients.items():
            if not client.wait_for_service(timeout_sec=60.0):
                log.seek(0)
                pytest.fail(f"service not available: {name}\n{log.read()[-4000:]}")
        yield node, clients
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGINT)
            process.wait(timeout=15)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass
            process.wait(timeout=5)
        log.close()


def test_fpc_plan_and_stream(stack):
    node, clients = stack
    request = PlanPose.Request()
    request.target_pose = Pose(
        position=Point(x=0.2872, y=0.0, z=0.5223),
        orientation=Quaternion(x=0.995, y=-0.0073, z=-0.0998, w=0.0007),
    )
    planned = _call(node, clients["plan"], request)
    assert planned.success, planned.message
    assert len(planned.trajectory) > 1
    assert planned.dt > 0.0

    sent = _call(node, clients["execute"], Trigger.Request())
    assert sent.success, sent.message

    deadline = time.monotonic() + 20.0
    while time.monotonic() < deadline:
        status = _call(node, clients["status"], Trigger.Request(), timeout_sec=10.0)
        if "completed" in status.message:
            return
        assert "canceled" not in status.message, status.message
        time.sleep(0.2)
    pytest.fail("FPC stream did not report completed")
