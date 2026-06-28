#!/usr/bin/env bash
# Apply the RobotOps Trace boundary patch to a stock ros2_controllers checkout,
# selecting the per-distro variant by ${ROS_DISTRO}.
#
# Usage: ./apply.sh /path/to/ros2_controllers [distro]
#        ROS_DISTRO=humble ./apply.sh /path/to/ros2_controllers
#
# The patch wires the RT-safe FollowJointTrajectoryTracer helper into
# joint_trajectory_controller's action-server boundary. Each distro variant is
# authored against the EXACT upstream ref that the apt deb for that distro is
# built from (see PIN below), so the rebuilt controller matches the underlay and
# `git apply --check` fails loudly on drift (CI re-checks per distro).
#
#   distro   upstream ros2_controllers ref the variant applies to (PIN)
#   ------   ----------------------------------------------------------
#   jazzy    tag 4.40.1   (Ubuntu 24.04 Noble; apt ros-jazzy-* == 4.40.1)
#   humble   tag 2.53.1   (Ubuntu 22.04 Jammy; apt ros-humble-* == 2.53.1)
#
# NOTE (ROB-449): the jazzy variant is pinned to the immutable 4.40.1 *tag* — the
# version apt actually ships — NOT the moving `jazzy` branch the original patch
# silently tracked (that branch had drifted past 4.40.1, e.g. preempt_active_goal
# became setAborted+runNonRealtime; the tag still uses setCanceled). Pinning the
# tag keeps the rebuild byte-aligned with the underlay and reproducible.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

TARGET="${1:-}"
DISTRO="${2:-${ROS_DISTRO:-}}"

declare -A PIN=( [jazzy]="4.40.1" [humble]="2.53.1" )

if [[ -z "${TARGET}" || ! -d "${TARGET}/joint_trajectory_controller" ]]; then
  echo "error: pass the path to a ros2_controllers checkout (containing joint_trajectory_controller/)" >&2
  echo "usage: $0 /path/to/ros2_controllers [distro]" >&2
  exit 2
fi
if [[ -z "${DISTRO}" ]]; then
  echo "error: no distro given (pass as arg 2 or set ROS_DISTRO). Known: ${!PIN[*]}" >&2
  exit 2
fi

PATCH="${HERE}/${DISTRO}/0001-joint-trajectory-controller-robotops-trace-boundary.patch"
if [[ ! -f "${PATCH}" ]]; then
  echo "error: no patch for distro '${DISTRO}' (looked for ${PATCH}). Known: ${!PIN[*]}" >&2
  exit 2
fi

echo "Distro '${DISTRO}': patch authored against ros2_controllers ${PIN[$DISTRO]:-?}."
echo "Checking patch applies cleanly to ${TARGET} ..."
git -C "${TARGET}" apply --check --verbose "${PATCH}"

echo "Applying ..."
git -C "${TARGET}" apply "${PATCH}"

echo "OK: joint_trajectory_controller patched (${DISTRO}). Build it against the apt"
echo "    underlay plus this workspace's robotops_trace_ros2_control install."
