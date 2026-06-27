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
RobotOps Trace — robotics semantic conventions v0 (ROB-430), Python mirror.

Byte-for-byte mirror of ``include/robotops_trace_semconv/semconv.hpp`` (the
authoritative source of truth, with the human-readable dictionary in
``robotics-semantic-conventions-v0.md``). Keeping the C++ header and this module
in lockstep is the contract that stops attribute names from drifting across the
rclpy integration and the pure-Python SDK core. Ships to PyPI as
``robotops-trace-semconv`` so pip-only users get the keys too.

Two namespaces (see the design doc):
  * ``robot.*`` — robotics-general concept keys, portable across frameworks.
  * ``ros.*``   — ROS mapping / implementation keys (present only when ROS).
  * resource attributes (``service.name``, ``robot.id``) — set once per process.

v0 is additive-only: new keys may be added; existing key names and value enums
are stable. Breaking a key name/enum = a new major.
"""

# ---------------------------------------------------------------------------
# Resource attributes (set once per process on the OTel resource, not per span)
# ---------------------------------------------------------------------------
SERVICE_NAME = 'service.name'
ROBOT_ID = 'robot.id'

# ---------------------------------------------------------------------------
# robot.* — concept keys (portable across frameworks)
# ---------------------------------------------------------------------------
# Action (ROS mapping: rclcpp_action / rclpy actions)
ROBOT_ACTION_NAME = 'robot.action.name'
ROBOT_ACTION_GOAL_ID = 'robot.action.goal_id'
ROBOT_ACTION_STATUS = 'robot.action.status'
ROBOT_ACTION_RESULT = 'robot.action.result'

# Work boundary (ROS mapping: executor callback)
ROBOT_CALLBACK_TYPE = 'robot.callback.type'

# Transform (ROS mapping: tf2)
ROBOT_TRANSFORM_PARENT = 'robot.transform.parent'
ROBOT_TRANSFORM_CHILD = 'robot.transform.child'

# Joint (ROS mapping: sensor_msgs/JointState, ros2_control)
ROBOT_JOINT_NAME = 'robot.joint.name'
ROBOT_JOINT_COUNT = 'robot.joint.count'

# Trajectory (ROS mapping: trajectory_msgs, FollowJointTrajectory)
ROBOT_TRAJECTORY_POINT_COUNT = 'robot.trajectory.point_count'
ROBOT_TRAJECTORY_DURATION_MS = 'robot.trajectory.duration_ms'

# Target / pose (ROS mapping: geometry_msgs/PoseStamped)
ROBOT_TARGET_FRAME = 'robot.target.frame'
ROBOT_TARGET_POSITION = 'robot.target.position'
ROBOT_TARGET_ORIENTATION = 'robot.target.orientation'

# Object (manipulation)
ROBOT_OBJECT_ID = 'robot.object.id'

# Component (ROS mapping: node)
ROBOT_COMPONENT_NAME = 'robot.component.name'

# ---------------------------------------------------------------------------
# ros.* — ROS mapping keys (implementation-specific; present only when ROS)
# ---------------------------------------------------------------------------
ROS_NODE = 'ros.node'
ROS_TOPIC = 'ros.topic'
ROS_SERVICE = 'ros.service'
ROS_MESSAGE_TYPE = 'ros.message.type'
ROS_PUBLISHER_GID = 'ros.publisher_gid'
ROS_SOURCE_TIMESTAMP = 'ros.source_timestamp'
ROS_MESSAGE_CONTENT_HASH = 'ros.message.content_hash'

# ---------------------------------------------------------------------------
# Enumerated string values (lowercase, stable). Use these instead of literals.
# ---------------------------------------------------------------------------
# Values for ROBOT_ACTION_STATUS: the goal lifecycle.
ROBOT_ACTION_STATUS_ACCEPTED = 'accepted'
ROBOT_ACTION_STATUS_EXECUTING = 'executing'
ROBOT_ACTION_STATUS_SUCCEEDED = 'succeeded'
ROBOT_ACTION_STATUS_ABORTED = 'aborted'
ROBOT_ACTION_STATUS_CANCELED = 'canceled'

# Values for ROBOT_ACTION_RESULT: the terminal domain outcome.
ROBOT_ACTION_RESULT_SUCCEEDED = 'succeeded'
ROBOT_ACTION_RESULT_ABORTED = 'aborted'
ROBOT_ACTION_RESULT_CANCELED = 'canceled'

# Values for ROBOT_CALLBACK_TYPE: the unit-of-work kind.
ROBOT_CALLBACK_TYPE_SUBSCRIPTION = 'subscription'
ROBOT_CALLBACK_TYPE_TIMER = 'timer'
ROBOT_CALLBACK_TYPE_SERVICE = 'service'
ROBOT_CALLBACK_TYPE_ACTION = 'action'
ROBOT_CALLBACK_TYPE_CLIENT = 'client'

# Schema version of this convention set (mirrors kSchemaVersion). Tracks the
# package version.
SCHEMA_VERSION = '0.2.0'

__all__ = [
    # resource
    'SERVICE_NAME',
    'ROBOT_ID',
    # robot.*
    'ROBOT_ACTION_NAME',
    'ROBOT_ACTION_GOAL_ID',
    'ROBOT_ACTION_STATUS',
    'ROBOT_ACTION_RESULT',
    'ROBOT_CALLBACK_TYPE',
    'ROBOT_TRANSFORM_PARENT',
    'ROBOT_TRANSFORM_CHILD',
    'ROBOT_JOINT_NAME',
    'ROBOT_JOINT_COUNT',
    'ROBOT_TRAJECTORY_POINT_COUNT',
    'ROBOT_TRAJECTORY_DURATION_MS',
    'ROBOT_TARGET_FRAME',
    'ROBOT_TARGET_POSITION',
    'ROBOT_TARGET_ORIENTATION',
    'ROBOT_OBJECT_ID',
    'ROBOT_COMPONENT_NAME',
    # ros.*
    'ROS_NODE',
    'ROS_TOPIC',
    'ROS_SERVICE',
    'ROS_MESSAGE_TYPE',
    'ROS_PUBLISHER_GID',
    'ROS_SOURCE_TIMESTAMP',
    'ROS_MESSAGE_CONTENT_HASH',
    # enum values
    'ROBOT_ACTION_STATUS_ACCEPTED',
    'ROBOT_ACTION_STATUS_EXECUTING',
    'ROBOT_ACTION_STATUS_SUCCEEDED',
    'ROBOT_ACTION_STATUS_ABORTED',
    'ROBOT_ACTION_STATUS_CANCELED',
    'ROBOT_ACTION_RESULT_SUCCEEDED',
    'ROBOT_ACTION_RESULT_ABORTED',
    'ROBOT_ACTION_RESULT_CANCELED',
    'ROBOT_CALLBACK_TYPE_SUBSCRIPTION',
    'ROBOT_CALLBACK_TYPE_TIMER',
    'ROBOT_CALLBACK_TYPE_SERVICE',
    'ROBOT_CALLBACK_TYPE_ACTION',
    'ROBOT_CALLBACK_TYPE_CLIENT',
    # version
    'SCHEMA_VERSION',
]
