#!/usr/bin/env bash
# SPDX-FileCopyrightText: Copyright (c) 2026 lzhshq
# SPDX-License-Identifier: Apache-2.0
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD="${ROOT}/../curobo_ros_ws/build"
ARTIFACT_ROOT="${CUROBO_ARTIFACT_ROOT:-/home/vitai/curobo-fast/cpp_runtime/robot/artifacts}"

if [[ ! -x "${BUILD}/curobo_core_cpp/plan_pose_smoke" ]]; then
  echo "build the workspace first:" >&2
  echo "  cd ~/curobo_ros_ws && colcon build --symlink-install --packages-up-to curobo_planner_cpp" >&2
  exit 1
fi

for robot in franka ar5; do
  artifact="${ARTIFACT_ROOT}/${robot}"
  if [[ ! -f "${artifact}/robot_kinematics_manifest.json" ]]; then
    echo "skip ${robot}: missing ${artifact}" >&2
    continue
  fi
  echo "== ${robot}: plan_pose_smoke =="
  "${BUILD}/curobo_core_cpp/plan_pose_smoke" "${artifact}"
  echo "== ${robot}: scene_smoke =="
  "${BUILD}/curobo_core_cpp/scene_smoke" "${artifact}"
done
