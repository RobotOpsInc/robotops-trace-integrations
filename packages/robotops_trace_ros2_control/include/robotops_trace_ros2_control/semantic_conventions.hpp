// Copyright 2026 Robot Ops Inc.
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

#ifndef ROBOTOPS_TRACE_ROS2_CONTROL__SEMANTIC_CONVENTIONS_HPP_
#define ROBOTOPS_TRACE_ROS2_CONTROL__SEMANTIC_CONVENTIONS_HPP_

/// \file semantic_conventions.hpp
/// \brief Span-attribute keys used by the ros2_control integration.
///
/// The authoritative, cross-framework key registry is `robotops_trace_semconv`
/// (ROB-430). Every key this integration emits has a portable concept-level
/// meaning, so they are all RE-EXPORTED (not redefined) from that header — the
/// emitted attribute strings are byte-identical to every other RobotOps
/// integration and to the ROSQL/agent vocabulary. There are NO ros2_control-local
/// keys: the FollowJointTrajectory boundary is fully described by the existing
/// action + joint + trajectory keys. Do NOT add keys here; promote them in
/// semconv (a deliberate, reviewed dictionary change) instead.

#include <robotops_trace_semconv/semconv.hpp>

namespace robotops::trace::ros2_control::keys
{

namespace semconv = ::robotops::trace::semconv;

// --- Action (the cross-process correlation key + lifecycle/outcome) ---------

/// Action name, e.g. "/arm_controller/follow_joint_trajectory". [str]
inline constexpr const char * kRobotActionName = semconv::kRobotActionName;
/// Goal UUID, RFC-4122 8-4-4-4-12 lowercase. The deterministic cross-process
/// join key (ROB-427); emitted identically on the action client + this server,
/// so the agent stitches the controller hop under the action client. [str]
inline constexpr const char * kRobotActionGoalId = semconv::kRobotActionGoalId;
/// Terminal domain outcome enum (succeeded | aborted | canceled). [str enum]
inline constexpr const char * kRobotActionResult = semconv::kRobotActionResult;

// --- Joint (from the goal's trajectory) -------------------------------------

/// Joint name(s) involved. Semconv types this as str[]; the SDK core 0.3.0
/// AttributeValue is scalar-only ("arrays come later"), so until the core ships
/// array attributes we emit the joint names as a single comma-joined string
/// under this key. The count below is the unambiguous queryable cardinality. [str]
inline constexpr const char * kRobotJointName = semconv::kRobotJointName;
/// Number of joints in the goal trajectory. [int64]
inline constexpr const char * kRobotJointCount = semconv::kRobotJointCount;

// --- Trajectory (from the goal's trajectory) --------------------------------

/// Number of trajectory points in the goal. [int64]
inline constexpr const char * kRobotTrajectoryPointCount =
  semconv::kRobotTrajectoryPointCount;

// --- Action result enum values (re-exported) --------------------------------

namespace action_result = semconv::action_result;

}  // namespace robotops::trace::ros2_control::keys

#endif  // ROBOTOPS_TRACE_ROS2_CONTROL__SEMANTIC_CONVENTIONS_HPP_
