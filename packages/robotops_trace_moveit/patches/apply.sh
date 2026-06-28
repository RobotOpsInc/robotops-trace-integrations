#!/usr/bin/env bash
# Apply the RobotOps Trace async-boundary patch to a stock moveit2 checkout.
#
# Usage: ./apply.sh /path/to/moveit2
#
# The patch is authored against moveit2 2.12.4 (jazzy) — specifically
# moveit_ros/planning's TrajectoryExecutionManager. `git apply` (used here)
# verifies it applies cleanly and fails loudly otherwise, so CI can detect
# upstream drift and the patch can be re-based.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PATCH="${HERE}/0001-trajectory-execution-manager-context-capture.patch"

TARGET="${1:-}"
if [[ -z "${TARGET}" || ! -d "${TARGET}/moveit_ros/planning/trajectory_execution_manager" ]]; then
  echo "error: pass the path to a moveit2 checkout (containing moveit_ros/planning/trajectory_execution_manager/)" >&2
  echo "usage: $0 /path/to/moveit2" >&2
  exit 2
fi

echo "Checking patch applies cleanly to ${TARGET} ..."
git -C "${TARGET}" apply --check --verbose "${PATCH}"

echo "Applying ..."
git -C "${TARGET}" apply "${PATCH}"

echo "OK: moveit_ros_planning patched. Build it against the apt MoveIt underlay"
echo "    plus this workspace's robotops_trace_moveit install."
