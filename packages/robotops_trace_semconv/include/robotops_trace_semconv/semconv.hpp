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
// RobotOps Trace — Semantic Conventions (ROB-430)  [STUB]
// ============================================================================
//
// Header-only registry of the robotics-general span/attribute keys shared by
// every RobotOps trace integration and the SDK cores. Keeping these in ONE
// header (mirrored by the python module) is the lockstep contract that stops
// attribute names from drifting across rclcpp/rclpy/bt_cpp/ros2_control/moveit.
//
// This is a SCAFFOLD. The keys below are concept-level placeholders drawn from
// the tracing architecture doc; the authoritative key set, value types, and the
// ROS-to-semconv mapping table are filled in by ROB-430.

#ifndef ROBOTOPS_TRACE_SEMCONV__SEMCONV_HPP_
#define ROBOTOPS_TRACE_SEMCONV__SEMCONV_HPP_

namespace robotops::trace::semconv {

// --- Concept-level robotics-general keys (TODO ROB-430: finalize) ----------
//
// TODO(ROB-430): robot.action.result        — terminal status of an action/goal
// TODO(ROB-430): robot.transform.parent      — TF parent frame id
// TODO(ROB-430): robot.transform.child       — TF child frame id
// TODO(ROB-430): robot.joint.name            — joint identifier
// TODO(ROB-430): robot.trajectory.point_count — number of points in a trajectory
//
// TODO(ROB-430): add the ROS-specific mapping (rclcpp_action goal UUID,
//                FollowJointTrajectory boundary, BT node name/uid, etc.)
// TODO(ROB-430): define value types + units and the namespacing scheme.

inline constexpr const char * kRobotActionResult = "robot.action.result";
inline constexpr const char * kRobotTransformParent = "robot.transform.parent";
inline constexpr const char * kRobotTransformChild = "robot.transform.child";
inline constexpr const char * kRobotJointName = "robot.joint.name";
inline constexpr const char * kRobotTrajectoryPointCount = "robot.trajectory.point_count";

// Schema version of this convention set. Bumped when keys are added/changed so
// consumers can assert compatibility. STUB value.
inline constexpr const char * kSchemaVersion = "0.1.0";

}  // namespace robotops::trace::semconv

#endif  // ROBOTOPS_TRACE_SEMCONV__SEMCONV_HPP_
