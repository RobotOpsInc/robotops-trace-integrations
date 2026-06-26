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
/// These are CONCEPT-LEVEL keys. The authoritative, cross-framework registry is
/// `robotops_trace_semconv` (ROB-430); until that package's key set is
/// finalized, the rclcpp integration carries local constants here.
///
/// TODO(ROB-430): delete these locals and import every key from
///   <robotops_trace_semconv/semconv.hpp>. The `robot.action.result` key is
///   ALREADY defined there and is re-exported below to prove the lockstep
///   contract works end-to-end; the rest are added to semconv by ROB-430.

#include <robotops_trace_semconv/semconv.hpp>

namespace robotops::trace::rclcpp::keys
{

// --- rclcpp_action: the deterministic cross-process correlation keys --------

/// The action goal UUID, canonical lowercase 8-4-4-4-12 form (see
/// `goal_id_to_string`). This is THE join key the correlation agent (ROB-427)
/// uses to stitch the client-side action span to the server-side action span
/// across processes. Emitted identically on both sides.
inline constexpr const char * kRobotActionGoalId = "robot.action.goal_id";

/// The action name (e.g. "/fibonacci").
inline constexpr const char * kRobotActionName = "robot.action.name";

/// Terminal result of a goal: "succeeded" | "aborted" | "canceled" | "unknown".
/// Re-exported from robotops_trace_semconv to demonstrate the ROB-430 contract.
inline constexpr const char * kRobotActionResult =
  ::robotops::trace::semconv::kRobotActionResult;

/// Server-side accept/reject decision: "accept_and_execute" | "accept_and_defer"
/// | "reject".
inline constexpr const char * kRobotActionGoalResponse =
  "robot.action.goal_response";

// --- topic/message content-correlation keys (best-effort, ROB-427) ----------

/// Topic name a subscription callback fired for.
inline constexpr const char * kRosTopic = "ros.topic";

/// Publisher GID (hex) of the received message — from rmw_message_info.
inline constexpr const char * kRosPublisherGid = "ros.publisher_gid";

/// Source (publish) timestamp in nanoseconds — from rmw_message_info.
inline constexpr const char * kRosSourceTimestamp = "ros.source_timestamp";

}  // namespace robotops::trace::rclcpp::keys

#endif  // ROBOTOPS_TRACE_RCLCPP__SEMANTIC_CONVENTIONS_HPP_
