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

#ifndef ROBOTOPS_TRACE_RCLCPP__ROBOTOPS_TRACE_RCLCPP_HPP_
#define ROBOTOPS_TRACE_RCLCPP__ROBOTOPS_TRACE_RCLCPP_HPP_

/// \file robotops_trace_rclcpp.hpp
/// \brief Umbrella include for the RobotOps Trace rclcpp integration (ROB-422).
///
/// Opt-in, fork-free instrumentation for C++ ROS 2 nodes:
///   - rclcpp_action client/server wrappers that emit the deterministic
///     goal-UUID correlation key (action_server.hpp, action_client.hpp);
///   - per-callback span wrappers + message content keys (callbacks.hpp).
///
/// All built on the transport-agnostic `robotops_trace_cpp` SDK core.

#include "robotops_trace_rclcpp/action_client.hpp"   // send_traced_goal, trace_send_goal_options
#include "robotops_trace_rclcpp/action_server.hpp"   // create_traced_action_server, scoped_action_span
#include "robotops_trace_rclcpp/callbacks.hpp"        // traced_callback, traced_subscription
#include "robotops_trace_rclcpp/identifiers.hpp"      // goal_id_to_string, gid_to_string
#include "robotops_trace_rclcpp/semantic_conventions.hpp"  // keys::*

namespace robotops::trace::rclcpp
{

/// Returns the package version string (matches package.xml).
const char * version() noexcept;

}  // namespace robotops::trace::rclcpp

#endif  // ROBOTOPS_TRACE_RCLCPP__ROBOTOPS_TRACE_RCLCPP_HPP_
