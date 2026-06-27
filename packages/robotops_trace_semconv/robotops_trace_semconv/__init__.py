# Copyright 2025 Robot Ops Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
RobotOps Trace semantic conventions (ROB-430) — Python mirror [STUB].

Mirror of ``include/robotops_trace_semconv/semconv.hpp``. Keeping the C++ header
and this module in lockstep is the contract that stops attribute names from
drifting across the rclpy integration and the pure-Python SDK core. Ships to
PyPI alongside ``robotops-trace-rclpy`` so pip-only users get the keys too.

This is a SCAFFOLD. Concept-level placeholder keys only; the authoritative set
and the ROS mapping land in ROB-430.
"""

# --- Concept-level robotics-general keys (TODO ROB-430: finalize) -----------
#
# TODO(ROB-430): keep this list byte-for-byte aligned with semconv.hpp.
# TODO(ROB-430): add ROS-specific mapping + value types/units.

ROBOT_ACTION_RESULT = 'robot.action.result'
ROBOT_TRANSFORM_PARENT = 'robot.transform.parent'
ROBOT_TRANSFORM_CHILD = 'robot.transform.child'
ROBOT_JOINT_NAME = 'robot.joint.name'
ROBOT_TRAJECTORY_POINT_COUNT = 'robot.trajectory.point_count'

# Schema version of this convention set (mirrors kSchemaVersion). STUB value.
SCHEMA_VERSION = '0.1.0'

__all__ = [
    'ROBOT_ACTION_RESULT',
    'ROBOT_TRANSFORM_PARENT',
    'ROBOT_TRANSFORM_CHILD',
    'ROBOT_JOINT_NAME',
    'ROBOT_TRAJECTORY_POINT_COUNT',
    'SCHEMA_VERSION',
]
