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

#ifndef ROBOTOPS_TRACE_RCLCPP__ACTION_CLIENT_HPP_
#define ROBOTOPS_TRACE_RCLCPP__ACTION_CLIENT_HPP_

#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robotops_trace/trace.hpp"

#include "robotops_trace_rclcpp/identifiers.hpp"
#include "robotops_trace_rclcpp/semantic_conventions.hpp"

/// \file action_client.hpp
/// \brief Traced rclcpp_action client wrappers — the deterministic goal-UUID key.
///
/// No fork of rclcpp_action: these wrap the PUBLIC `SendGoalOptions` callbacks.
/// The client side emits `robot.action.goal_id` (the same canonical goal UUID
/// the server emits) so the correlation agent (ROB-427) joins the two traces.

namespace robotops::trace::rclcpp
{

namespace detail
{

inline const char * result_code_str(::rclcpp_action::ResultCode c) noexcept
{
  switch (c) {
    case ::rclcpp_action::ResultCode::SUCCEEDED: return "succeeded";
    case ::rclcpp_action::ResultCode::ABORTED: return "aborted";
    case ::rclcpp_action::ResultCode::CANCELED: return "canceled";
    case ::rclcpp_action::ResultCode::UNKNOWN: return "unknown";
  }
  return "unknown";
}

}  // namespace detail

/// Wrap a `SendGoalOptions` so the goal's lifecycle is traced.
///
/// Mechanism + thread-safety: the RobotOps SDK's span records are thread-
/// confined (RAII on the thread that opened them). An action goal is inherently
/// async — the result arrives on an executor thread, not the thread that called
/// `async_send_goal`. So we do NOT hold one span open across that boundary
/// (which would pop the thread-local stack on the wrong thread). Instead we:
///   1. capture the caller's active context SYNCHRONOUSLY here, and
///   2. open + close a span ENTIRELY WITHIN each async callback, parented to the
///      captured context via SpanOptions::parent.
/// The result is a span carrying the goal UUID, correctly nested under the
/// caller's trace, created safely on whatever thread the callback runs.
///
/// Spans produced (when the corresponding event fires):
///   - "<name> action.goal_response" (Client) — accept/reject, on response.
///   - "<name> action.result"        (Client) — terminal result + status.
/// Any user callbacks you set are preserved and invoked inside the span scope.
template<typename ActionT>
typename ::rclcpp_action::Client<ActionT>::SendGoalOptions trace_send_goal_options(
  std::string action_name,
  typename ::rclcpp_action::Client<ActionT>::SendGoalOptions user_options = {})
{
  using GoalHandle = ::rclcpp_action::ClientGoalHandle<ActionT>;

  // Capture-on-submit: snapshot the active context on the CALLING thread.
  const ::robotops::Context parent = ::robotops::capture_context();

  // --- goal response (accepted / rejected) ---
  auto user_response_cb = std::move(user_options.goal_response_callback);
  user_options.goal_response_callback =
    [action_name, parent, user_response_cb](typename GoalHandle::SharedPtr handle)
    {
      ::robotops::SpanOptions opts;
      opts.kind = ::robotops::SpanKind::Client;
      const ::robotops::SpanContext parent_ctx = parent.span_context();
      if (parent_ctx.valid()) {
        opts.parent = &parent_ctx;
      }
      ::robotops::SpanGuard guard(action_name + " action.goal_response", opts);
      auto span = guard.span();
      span.set_attribute(keys::kRobotActionName, action_name);
      const bool accepted = static_cast<bool>(handle);
      span.set_attribute("robot.action.goal_accepted", accepted);
      if (accepted) {
        span.set_attribute(keys::kRobotActionGoalId, goal_id_to_string(handle->get_goal_id()));
      }
      if (user_response_cb) {
        user_response_cb(handle);
      }
    };

  // --- terminal result ---
  auto user_result_cb = std::move(user_options.result_callback);
  user_options.result_callback =
    [action_name, parent, user_result_cb](const typename GoalHandle::WrappedResult & result)
    {
      ::robotops::SpanOptions opts;
      opts.kind = ::robotops::SpanKind::Client;
      const ::robotops::SpanContext parent_ctx = parent.span_context();
      if (parent_ctx.valid()) {
        opts.parent = &parent_ctx;
      }
      ::robotops::SpanGuard guard(action_name + " action.result", opts);
      auto span = guard.span();
      span.set_attribute(keys::kRobotActionGoalId, goal_id_to_string(result.goal_id));
      span.set_attribute(keys::kRobotActionName, action_name);
      span.set_attribute(keys::kRobotActionResult, detail::result_code_str(result.code));
      span.set_status(
        result.code == ::rclcpp_action::ResultCode::SUCCEEDED ?
        ::robotops::StatusCode::Ok : ::robotops::StatusCode::Error);
      if (user_result_cb) {
        user_result_cb(result);
      }
    };

  return user_options;
}

/// Convenience: send a goal through a traced `SendGoalOptions`. Equivalent to
/// `client->async_send_goal(goal, trace_send_goal_options<ActionT>(name, opts))`.
template<typename ActionT>
auto send_traced_goal(
  const std::shared_ptr<::rclcpp_action::Client<ActionT>> & client,
  const typename ActionT::Goal & goal,
  std::string action_name,
  typename ::rclcpp_action::Client<ActionT>::SendGoalOptions user_options = {})
{
  return client->async_send_goal(
    goal, trace_send_goal_options<ActionT>(std::move(action_name), std::move(user_options)));
}

}  // namespace robotops::trace::rclcpp

#endif  // ROBOTOPS_TRACE_RCLCPP__ACTION_CLIENT_HPP_
