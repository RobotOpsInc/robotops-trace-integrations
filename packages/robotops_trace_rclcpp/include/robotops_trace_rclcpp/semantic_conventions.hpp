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

#ifndef ROBOTOPS_TRACE_RCLCPP__SEMANTIC_CONVENTIONS_HPP_
#define ROBOTOPS_TRACE_RCLCPP__SEMANTIC_CONVENTIONS_HPP_

/// \file semantic_conventions.hpp
/// \brief Span-attribute keys used by the rclcpp integration.
///
/// As of ROB-430 the authoritative, cross-framework key registry is
/// `robotops_trace_semconv`. The keys below are RE-EXPORTED (not redefined) from
/// that header, so the emitted attribute strings are guaranteed byte-identical
/// to every other RobotOps integration and to the ROSQL/agent vocabulary. The
/// `keys::` alias is kept for source compatibility; reach for the semconv
/// constants directly in new code.

#include <robotops_trace_semconv/semconv.hpp>

namespace robotops::trace::rclcpp::keys
{

namespace semconv = ::robotops::trace::semconv;

// --- keys sourced from the authoritative semconv dictionary (ROB-430) -------

/// The action goal UUID, canonical lowercase 8-4-4-4-12 form (see
/// `goal_id_to_string`). THE join key the correlation agent (ROB-427) uses to
/// stitch the client-side action span to the server-side action span across
/// processes. Emitted identically on both sides.
inline constexpr const char * kRobotActionGoalId = semconv::kRobotActionGoalId;

/// The action name (e.g. "/fibonacci").
inline constexpr const char * kRobotActionName = semconv::kRobotActionName;

/// Terminal domain result of a goal (see semconv::action_result values).
inline constexpr const char * kRobotActionResult = semconv::kRobotActionResult;

/// Topic name a subscription callback fired for.
inline constexpr const char * kRosTopic = semconv::kRosTopic;

/// Publisher GID (hex) of the received message — from rmw_message_info.
inline constexpr const char * kRosPublisherGid = semconv::kRosPublisherGid;

/// Source (publish) timestamp in nanoseconds — from rmw_message_info.
inline constexpr const char * kRosSourceTimestamp = semconv::kRosSourceTimestamp;

// --- rclcpp-local keys NOT (yet) in semconv v0 ------------------------------
//
// The dictionary is intentionally minimal and only promotes a key once it has a
// stable cross-framework meaning. These describe rclcpp_action's server-side
// accept decision, which has no portable concept-level analogue yet; they stay
// local here until/unless a future semconv minor adopts them.

/// Server-side accept/reject decision: "accept_and_execute" | "accept_and_defer"
/// | "reject".
inline constexpr const char * kRobotActionGoalResponse =
  "robot.action.goal_response";

/// Whether the server accepted the goal (client-observed). [bool attribute]
inline constexpr const char * kRobotActionGoalAccepted =
  "robot.action.goal_accepted";

}  // namespace robotops::trace::rclcpp::keys

#endif  // ROBOTOPS_TRACE_RCLCPP__SEMANTIC_CONVENTIONS_HPP_
