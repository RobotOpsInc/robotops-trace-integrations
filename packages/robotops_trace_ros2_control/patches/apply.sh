#!/usr/bin/env bash
# Apply the RobotOps Trace boundary patch to a stock ros2_controllers checkout.
#
# Usage: ./apply.sh /path/to/ros2_controllers
#
# The patch is authored against ros2_controllers 4.40.1 (jazzy). `git apply`
# (used here) verifies it applies cleanly and fails loudly otherwise, so CI can
# detect upstream drift and the patch can be re-based.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PATCH="${HERE}/0001-joint-trajectory-controller-robotops-trace-boundary.patch"

TARGET="${1:-}"
if [[ -z "${TARGET}" || ! -d "${TARGET}/joint_trajectory_controller" ]]; then
  echo "error: pass the path to a ros2_controllers checkout (containing joint_trajectory_controller/)" >&2
  echo "usage: $0 /path/to/ros2_controllers" >&2
  exit 2
fi

echo "Checking patch applies cleanly to ${TARGET} ..."
git -C "${TARGET}" apply --check --verbose "${PATCH}"

echo "Applying ..."
git -C "${TARGET}" apply "${PATCH}"

echo "OK: joint_trajectory_controller patched. Build it against the apt underlay"
echo "    plus this workspace's robotops_trace_ros2_control install."
