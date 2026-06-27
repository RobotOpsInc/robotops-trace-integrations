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

#ifndef ROBOTOPS_TRACE_RCLCPP__ACTION_SERVER_HPP_
#define ROBOTOPS_TRACE_RCLCPP__ACTION_SERVER_HPP_

#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robotops_trace/trace.hpp"

#include "robotops_trace_rclcpp/identifiers.hpp"
#include "robotops_trace_rclcpp/semantic_conventions.hpp"

/// \file action_server.hpp
/// \brief Traced rclcpp_action server wrappers — the deterministic goal-UUID key.
///
/// No fork of rclcpp_action: these are thin, opt-in wrappers around the PUBLIC
/// `rclcpp_action::create_server` callbacks. Each goal produces server-side
/// spans carrying `robot.action.goal_id` (the canonical goal UUID) so the
/// correlation agent (ROB-427) can join the server trace to the client trace
/// across the process boundary.

namespace robotops::trace::rclcpp
{

namespace detail
{

inline const char * goal_response_str(::rclcpp_action::GoalResponse r) noexcept
{
  switch (r) {
    case ::rclcpp_action::GoalResponse::REJECT: return "reject";
    case ::rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE: return "accept_and_execute";
    case ::rclcpp_action::GoalResponse::ACCEPT_AND_DEFER: return "accept_and_defer";
  }
  return "unknown";
}

}  // namespace detail

/// Open a Server-kind span covering an action goal's execution, keyed by the
/// goal UUID. Call this at the top of your server's `execute()` method and hold
/// the returned guard for the execution scope; it closes (and the span is
/// emitted) when the guard goes out of scope.
///
/// This is the RECOMMENDED server primitive: because rclcpp executes a goal on
/// whatever thread the user drives (often one spawned from `handle_accepted`),
/// the only place a single span can deterministically cover the *whole* unit of
/// work is inside that synchronous execute body. The span carries the goal UUID,
/// so it correlates cross-process regardless of which thread runs it.
///
/// \code
///   void execute(const std::shared_ptr<GoalHandleFib> goal_handle) {
///     auto span = robotops::trace::rclcpp::scoped_action_span(goal_handle, "/fibonacci");
///     // ... do the work ...
///     goal_handle->succeed(result);
///     span.span().set_attribute(keys::kRobotActionResult, "succeeded");
///   }
/// \endcode
template<typename ActionT>
::robotops::SpanGuard scoped_action_span(
  const std::shared_ptr<::rclcpp_action::ServerGoalHandle<ActionT>> & goal_handle,
  std::string action_name)
{
  ::robotops::SpanOptions opts;
  opts.kind = ::robotops::SpanKind::Server;
  ::robotops::SpanGuard guard(action_name + " action.execute", opts);
  auto span = guard.span();
  span.set_attribute(keys::kRobotActionGoalId, goal_id_to_string(goal_handle->get_goal_id()));
  span.set_attribute(keys::kRobotActionName, action_name);
  return guard;  // move-out; RAII closes the span at the caller's scope end
}

/// Drop-in replacement for `rclcpp_action::create_server` that wraps the three
/// server callbacks so each goal automatically produces spans carrying the
/// goal-UUID key — no changes to the callback bodies.
///
/// What it instruments:
///  - `handle_goal`     -> a synchronous Server span "<name> action.goal_request"
///                         carrying `robot.action.goal_id` and the accept/reject
///                         decision. Fully covers the (synchronous) decision.
///  - `handle_cancel`   -> a synchronous Server span "<name> action.cancel".
///  - `handle_accepted` -> a Server span "<name> action.accepted" carrying the
///                         goal UUID, covering the (synchronous) dispatch.
///
/// LIMITATION: `handle_accepted` typically just spawns the execution thread and
/// returns, so its span covers dispatch, not the work. For a span over the full
/// execution, call `scoped_action_span()` inside your `execute()` body (above).
template<typename ActionT, typename NodeT>
typename ::rclcpp_action::Server<ActionT>::SharedPtr create_traced_action_server(
  NodeT node,
  const std::string & name,
  typename ::rclcpp_action::Server<ActionT>::GoalCallback handle_goal,
  typename ::rclcpp_action::Server<ActionT>::CancelCallback handle_cancel,
  typename ::rclcpp_action::Server<ActionT>::AcceptedCallback handle_accepted,
  const rcl_action_server_options_t & options = rcl_action_server_get_default_options(),
  ::rclcpp::CallbackGroup::SharedPtr group = nullptr)
{
  using GoalUUID = ::rclcpp_action::GoalUUID;
  using Goal = typename ActionT::Goal;
  using GoalHandle = ::rclcpp_action::ServerGoalHandle<ActionT>;

  auto traced_goal =
    [name, handle_goal = std::move(handle_goal)](
    const GoalUUID & uuid, std::shared_ptr<const Goal> goal)
    {
      ::robotops::SpanOptions opts;
      opts.kind = ::robotops::SpanKind::Server;
      ::robotops::SpanGuard guard(name + " action.goal_request", opts);
      auto span = guard.span();
      span.set_attribute(keys::kRobotActionGoalId, goal_id_to_string(uuid));
      span.set_attribute(keys::kRobotActionName, name);
      const auto response = handle_goal(uuid, std::move(goal));
      span.set_attribute(keys::kRobotActionGoalResponse, detail::goal_response_str(response));
      return response;
    };

  auto traced_cancel =
    [name, handle_cancel = std::move(handle_cancel)](std::shared_ptr<GoalHandle> goal_handle)
    {
      ::robotops::SpanOptions opts;
      opts.kind = ::robotops::SpanKind::Server;
      ::robotops::SpanGuard guard(name + " action.cancel", opts);
      auto span = guard.span();
      span.set_attribute(keys::kRobotActionGoalId, goal_id_to_string(goal_handle->get_goal_id()));
      span.set_attribute(keys::kRobotActionName, name);
      return handle_cancel(std::move(goal_handle));
    };

  auto traced_accepted =
    [name, handle_accepted = std::move(handle_accepted)](std::shared_ptr<GoalHandle> goal_handle)
    {
      ::robotops::SpanOptions opts;
      opts.kind = ::robotops::SpanKind::Server;
      ::robotops::SpanGuard guard(name + " action.accepted", opts);
      auto span = guard.span();
      span.set_attribute(keys::kRobotActionGoalId, goal_id_to_string(goal_handle->get_goal_id()));
      span.set_attribute(keys::kRobotActionName, name);
      handle_accepted(std::move(goal_handle));
    };

  return ::rclcpp_action::create_server<ActionT>(
    node, name, std::move(traced_goal), std::move(traced_cancel),
    std::move(traced_accepted), options, group);
}

}  // namespace robotops::trace::rclcpp

#endif  // ROBOTOPS_TRACE_RCLCPP__ACTION_SERVER_HPP_
