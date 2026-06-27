// Copyright 2025 Robot Ops Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// ============================================================================
// RobotOps Trace — Robotics Semantic Conventions v0 (ROB-430)
// ============================================================================
//
// Header-only registry of the robotics span/attribute keys shared by every
// RobotOps trace integration (rclcpp/rclpy/BT.CPP/ros2_control/MoveIt) and the
// SDK cores. This header (mirrored byte-for-byte by the Python module
// robotops_trace_semconv/__init__.py) is the AUTHORITATIVE source of truth; the
// human-readable dictionary lives in the design doc
// (tracehouse-mvp-planning/robotics-semantic-conventions-v0.md).
//
// Two namespaces (see design doc §"Design rules"):
//   * robot.* — robotics-GENERAL concept keys. Portable; any robot has actions,
//     transforms, joints, trajectories. This is the durable vocabulary
//     TraceHouse/ROSQL display + filter on.
//   * ros.*   — the ROS mapping / implementation keys (topic, gid, message
//     type). Present only when the transport IS ROS.
//   * resource attributes (service.name, robot.id) are set once per process on
//     the OTel resource, not per span.
//
// Types (OTel AttributeValue): string, bool, int64, double, or arrays thereof.
// No nested structs — poses are decomposed into double[] components.
//
// v0 is additive-only: new keys may be added; existing key names + value enums
// are stable. Breaking a key name/enum = a new major.

#ifndef ROBOTOPS_TRACE_SEMCONV__SEMCONV_HPP_
#define ROBOTOPS_TRACE_SEMCONV__SEMCONV_HPP_

namespace robotops::trace::semconv
{

// ===========================================================================
// Resource attributes (set once per process on the OTel resource, NOT per span)
// ===========================================================================

/// OTel-standard process/node identity. Reuse OTel's key; don't reinvent. [str]
inline constexpr const char * kServiceName = "service.name";
/// Robot identity (the agent already stamps RobotId; align the key). [str]
inline constexpr const char * kRobotId = "robot.id";

// ===========================================================================
// robot.* — concept keys (portable across frameworks)
// ===========================================================================

// --- Action (ROS mapping: rclcpp_action / rclpy actions) -------------------

/// Action name, e.g. "navigate_to_pose", "move_action". [str]
inline constexpr const char * kRobotActionName = "robot.action.name";
/// Goal UUID, RFC-4122 8-4-4-4-12 lowercase. The deterministic cross-process
/// join key (ROB-427); emitted identically on client + server. [str]
inline constexpr const char * kRobotActionGoalId = "robot.action.goal_id";
/// Lifecycle status enum (see action_status). [str enum]
inline constexpr const char * kRobotActionStatus = "robot.action.status";
/// Terminal domain outcome enum (see action_result), distinct from span
/// status. [str enum]
inline constexpr const char * kRobotActionResult = "robot.action.result";

// --- Work boundary (ROS mapping: executor callback) ------------------------

/// The universal "unit of work" kind enum (see callback_type). [str enum]
inline constexpr const char * kRobotCallbackType = "robot.callback.type";

// --- Transform (ROS mapping: tf2) ------------------------------------------

/// Parent frame id. [str]
inline constexpr const char * kRobotTransformParent = "robot.transform.parent";
/// Child frame id. [str]
inline constexpr const char * kRobotTransformChild = "robot.transform.child";

// --- Joint (ROS mapping: sensor_msgs/JointState, ros2_control) -------------

/// Joint name(s) involved. [str[]]
inline constexpr const char * kRobotJointName = "robot.joint.name";
/// Number of joints. [int64]
inline constexpr const char * kRobotJointCount = "robot.joint.count";

// --- Trajectory (ROS mapping: trajectory_msgs, FollowJointTrajectory) ------

/// Number of trajectory points. [int64]
inline constexpr const char * kRobotTrajectoryPointCount =
  "robot.trajectory.point_count";
/// Planned trajectory duration in milliseconds. [double]
inline constexpr const char * kRobotTrajectoryDurationMs =
  "robot.trajectory.duration_ms";

// --- Target / pose (ROS mapping: geometry_msgs/PoseStamped) ----------------

/// Frame id the target is expressed in. [str]
inline constexpr const char * kRobotTargetFrame = "robot.target.frame";
/// Target position [x, y, z] in meters — decomposed, not a struct. [double[]]
inline constexpr const char * kRobotTargetPosition = "robot.target.position";
/// Target orientation quaternion [x, y, z, w]. [double[]]
inline constexpr const char * kRobotTargetOrientation =
  "robot.target.orientation";

// --- Object (manipulation) -------------------------------------------------

/// Object/target identifier (e.g. a grasp target). [str]
inline constexpr const char * kRobotObjectId = "robot.object.id";

// --- Component (ROS mapping: node) -----------------------------------------

/// Logical component/node name. [str]
inline constexpr const char * kRobotComponentName = "robot.component.name";

// ===========================================================================
// ros.* — ROS mapping keys (implementation-specific; present only when ROS)
// ===========================================================================

/// ROS node name. [str]
inline constexpr const char * kRosNode = "ros.node";
/// Topic name. [str]
inline constexpr const char * kRosTopic = "ros.topic";
/// Service name. [str]
inline constexpr const char * kRosService = "ros.service";
/// ROS message/interface type, e.g. "nav2_msgs/action/NavigateToPose". [str]
inline constexpr const char * kRosMessageType = "ros.message.type";
/// DDS publisher GID (hex) — content-correlation key (ROB-427). [str]
inline constexpr const char * kRosPublisherGid = "ros.publisher_gid";
/// DDS source timestamp (ns) — content-correlation key. [int64]
inline constexpr const char * kRosSourceTimestamp = "ros.source_timestamp";
/// Content hash — best-effort content-correlation key. [str]
inline constexpr const char * kRosMessageContentHash =
  "ros.message.content_hash";

// ===========================================================================
// Enumerated string values (lowercase, stable). Use these instead of literals
// so producers cannot drift from the dictionary.
// ===========================================================================

/// Values for kRobotActionStatus: the goal lifecycle.
namespace action_status
{
inline constexpr const char * kAccepted = "accepted";
inline constexpr const char * kExecuting = "executing";
inline constexpr const char * kSucceeded = "succeeded";
inline constexpr const char * kAborted = "aborted";
inline constexpr const char * kCanceled = "canceled";
}  // namespace action_status

/// Values for kRobotActionResult: the terminal domain outcome.
namespace action_result
{
inline constexpr const char * kSucceeded = "succeeded";
inline constexpr const char * kAborted = "aborted";
inline constexpr const char * kCanceled = "canceled";
}  // namespace action_result

/// Values for kRobotCallbackType: the unit-of-work kind.
namespace callback_type
{
inline constexpr const char * kSubscription = "subscription";
inline constexpr const char * kTimer = "timer";
inline constexpr const char * kService = "service";
inline constexpr const char * kAction = "action";
inline constexpr const char * kClient = "client";
}  // namespace callback_type

// ===========================================================================
// Schema version of this convention set. Bumped when keys are added/changed so
// consumers can assert compatibility. Tracks the package version.
// ===========================================================================
inline constexpr const char * kSchemaVersion = "0.2.0";

}  // namespace robotops::trace::semconv

#endif  // ROBOTOPS_TRACE_SEMCONV__SEMCONV_HPP_
