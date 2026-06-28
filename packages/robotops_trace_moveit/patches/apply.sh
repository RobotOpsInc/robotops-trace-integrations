#!/usr/bin/env bash
# Apply the RobotOps Trace async-boundary patch to a stock moveit2 checkout,
# selecting the per-distro variant by ${ROS_DISTRO}.
#
# Usage: ./apply.sh /path/to/moveit2 [distro]
#        ROS_DISTRO=humble ./apply.sh /path/to/moveit2
#
# The patch wires the TrajectoryExecutionTracer capture/restore helper into
# moveit_ros/planning's TrajectoryExecutionManager (push / executePart / clear).
# Each distro variant is authored against the EXACT upstream moveit2 ref the apt
# deb for that distro is built from (see PIN below); `git apply --check` fails
# loudly on drift so CI can re-base per distro.
#
#   distro   upstream moveit2 ref the variant applies to (PIN)
#   ------   --------------------------------------------------
#   jazzy    tag 2.12.4   (Ubuntu 24.04 Noble; apt ros-jazzy-moveit-ros-planning == 2.12.4)
#   humble   tag 2.5.9    (Ubuntu 22.04 Jammy; apt ros-humble-moveit-ros-planning == 2.5.9)
#
# NOTE (ROB-449): the humble TEM diverges structurally from jazzy — the public
# header is trajectory_execution_manager.h (jazzy: .hpp) and the planning CMake
# uses ament_target_dependencies/THIS_PACKAGE_INCLUDE_DEPENDS rather than the
# modern target_link_libraries — so the humble variant is a real re-base, not a
# byte-copy. The helper API is identical across distros.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

TARGET="${1:-}"
DISTRO="${2:-${ROS_DISTRO:-}}"

declare -A PIN=( [jazzy]="2.12.4" [humble]="2.5.9" )

if [[ -z "${TARGET}" || ! -d "${TARGET}/moveit_ros/planning/trajectory_execution_manager" ]]; then
  echo "error: pass the path to a moveit2 checkout (containing moveit_ros/planning/trajectory_execution_manager/)" >&2
  echo "usage: $0 /path/to/moveit2 [distro]" >&2
  exit 2
fi
if [[ -z "${DISTRO}" ]]; then
  echo "error: no distro given (pass as arg 2 or set ROS_DISTRO). Known: ${!PIN[*]}" >&2
  exit 2
fi

PATCH="${HERE}/${DISTRO}/0001-trajectory-execution-manager-context-capture.patch"
if [[ ! -f "${PATCH}" ]]; then
  echo "error: no patch for distro '${DISTRO}' (looked for ${PATCH}). Known: ${!PIN[*]}" >&2
  exit 2
fi

echo "Distro '${DISTRO}': patch authored against moveit2 ${PIN[$DISTRO]:-?}."
echo "Checking patch applies cleanly to ${TARGET} ..."
git -C "${TARGET}" apply --check --verbose "${PATCH}"

echo "Applying ..."
git -C "${TARGET}" apply "${PATCH}"

echo "OK: moveit_ros_planning patched (${DISTRO}). Build it against the apt MoveIt"
echo "    underlay plus this workspace's robotops_trace_moveit install."
