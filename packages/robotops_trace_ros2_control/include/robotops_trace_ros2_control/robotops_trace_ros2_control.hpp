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

#ifndef ROBOTOPS_TRACE_ROS2_CONTROL__ROBOTOPS_TRACE_ROS2_CONTROL_HPP_
#define ROBOTOPS_TRACE_ROS2_CONTROL__ROBOTOPS_TRACE_ROS2_CONTROL_HPP_

/// \file robotops_trace_ros2_control.hpp
/// \brief Umbrella include for the RobotOps Trace ros2_control integration
/// (ROB-425).
///
/// RT-safe instrumentation of the controller / FollowJointTrajectory action-server
/// boundary: one detached SERVER span per goal, from accept to terminal result,
/// carrying the canonical goal UUID (the cross-process join key, ROB-427) plus the
/// joint/trajectory semantic conventions. Nothing here ever runs in the real-time
/// `update()` control loop. Built on the transport-agnostic `robotops_trace_cpp`
/// SDK core; attribute keys come from the shared `robotops_trace_semconv`
/// dictionary.

#include "robotops_trace_ros2_control/follow_joint_trajectory_tracer.hpp"  // FollowJointTrajectoryTracer
#include "robotops_trace_ros2_control/identifiers.hpp"                     // goal_id_to_string
#include "robotops_trace_ros2_control/semantic_conventions.hpp"           // keys::*

namespace robotops::trace::ros2_control
{

/// Returns the package version string (matches package.xml).
const char * version() noexcept;

}  // namespace robotops::trace::ros2_control

#endif  // ROBOTOPS_TRACE_ROS2_CONTROL__ROBOTOPS_TRACE_ROS2_CONTROL_HPP_
