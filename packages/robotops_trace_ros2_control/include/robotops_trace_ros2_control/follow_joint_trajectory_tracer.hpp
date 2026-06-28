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

#ifndef ROBOTOPS_TRACE_ROS2_CONTROL__FOLLOW_JOINT_TRAJECTORY_TRACER_HPP_
#define ROBOTOPS_TRACE_ROS2_CONTROL__FOLLOW_JOINT_TRAJECTORY_TRACER_HPP_

#include <mutex>
#include <string>
#include <unordered_map>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "rclcpp_action/types.hpp"
#include "robotops_trace/trace.hpp"

#include "robotops_trace_ros2_control/semantic_conventions.hpp"

/// \file follow_joint_trajectory_tracer.hpp
/// \brief `FollowJointTrajectoryTracer` — the RT-safe traced-boundary utility for
/// a FollowJointTrajectory action server.
///
/// WHAT IT IS. A small helper that opens ONE server-side span per action goal,
/// keyed by the goal UUID, covering the goal from accept to terminal result. It
/// is the reusable primitive that:
///   * the carried `joint_trajectory_controller` patch (patches/) wires into
///     stock JTC at the accept/result boundary, and
///   * a custom controller calls directly from its own action callbacks.
///
/// WHY A DETACHED SPAN. A goal is accepted on the executor thread and runs across
/// the controller's async execution (the RT update() loop + the non-RT action
/// monitor), then completes on the monitor thread — a different call stack from
/// where it opened. The SDK's thread-local current-context is therefore the WRONG
/// span-lifetime model. We use the core's detached-span API
/// (`robotops::start_detached_span`, ROB-443): it mints a span WITHOUT touching
/// any thread's current-context, the caller owns its lifetime, and the parent is
/// set EXPLICITLY. So the span can be held from accept to result regardless of
/// which thread observes the terminal.
///
/// RT-SAFETY (hard requirement). Every method here runs ONLY at the action-server
/// boundary on the executor / non-RT monitor thread — NEVER from the real-time
/// `update()` control loop. update() must contain zero span ops, zero allocation,
/// zero locking, zero logging (see the carried patch and the package README). The
/// terminal OUTCOME is decided inside update() the RT-safe ros2_control way (a
/// `RealtimeServerGoalHandle::setSucceeded/Aborted` flag write into a lock-free
/// buffer); the SPAN is closed later, on the non-RT thread, by `on_result()`.
///
/// ZERO-ROBOT-IMPACT. Every method is `noexcept` and wraps its body in a
/// catch-all; the underlying detached-span ops are themselves noexcept and
/// degrade to cheap no-ops when the SDK is disabled / uninitialised. A tracing
/// fault can never perturb the controller's goal handling.
///
/// THREAD-SAFETY. The active-goal map is guarded by a plain mutex. This lock is
/// taken ONLY on the non-RT boundary threads (a multi-threaded executor may run
/// accept and the monitor timer on different threads); it is never taken on the
/// RT path, so it does not affect control-loop determinism.

namespace robotops::trace::ros2_control
{

/// Opens/closes one detached SERVER span per FollowJointTrajectory goal.
///
/// Hold one instance for the lifetime of the controller (e.g. as a member). Call
/// `on_goal_accepted()` when a goal is accepted and `on_result()` when it reaches
/// a terminal state. Both run on the non-RT executor / monitor thread.
class FollowJointTrajectoryTracer
{
public:
  using Goal = ::control_msgs::action::FollowJointTrajectory::Goal;

  FollowJointTrajectoryTracer() = default;
  ~FollowJointTrajectoryTracer() = default;

  FollowJointTrajectoryTracer(const FollowJointTrajectoryTracer &) = delete;
  FollowJointTrajectoryTracer & operator=(const FollowJointTrajectoryTracer &) = delete;
  FollowJointTrajectoryTracer(FollowJointTrajectoryTracer &&) = delete;
  FollowJointTrajectoryTracer & operator=(FollowJointTrajectoryTracer &&) = delete;

  /// Open the goal's server span. Call from `handle_accepted` (non-RT).
  ///
  /// Emits, on a SERVER-kind detached span named "<action_name> follow_joint_trajectory":
  ///   * robot.action.goal_id   — the canonical RFC-4122 goal UUID (the join key)
  ///   * robot.action.name      — `action_name`
  ///   * robot.joint.name       — the goal's joint names (comma-joined; see semconv note)
  ///   * robot.joint.count      — number of joints
  ///   * robot.trajectory.point_count — number of trajectory points
  ///
  /// \param uuid        The goal UUID (from `goal_handle->get_goal_id()`).
  /// \param goal        The goal message (from `goal_handle->get_goal()`).
  /// \param action_name The action name (e.g. the controller's FJT action topic).
  /// \param parent      Optional EXPLICIT parent span context. Normally null: the
  ///   server span is the root of the server-side trace and is joined to the
  ///   client trace cross-process by goal_id (ROB-427), not by an in-process
  ///   parent. Pass a parent only when the controller already owns an enclosing
  ///   span it wants the goal nested under.
  void on_goal_accepted(
    const ::rclcpp_action::GoalUUID & uuid,
    const Goal & goal,
    const std::string & action_name,
    const ::robotops::SpanContext * parent = nullptr) noexcept;

  /// Close the goal's server span, stamping the terminal outcome. Call from the
  /// non-RT thread that observes the goal terminal (the action monitor timer for
  /// the success/abort path, `handle_cancel`/preempt for the cancel/preempt
  /// path). No-op if no span is open for `uuid` (idempotent across the monitor
  /// timer firing repeatedly).
  ///
  /// \param uuid   The goal UUID.
  /// \param result One of robotops::trace::semconv::action_result::{kSucceeded,
  ///   kAborted, kCanceled}. `succeeded` -> span StatusCode::Ok; `aborted` ->
  ///   StatusCode::Error; `canceled` -> StatusCode::Unset (neither a node success
  ///   nor an error of the server itself).
  void on_result(
    const ::rclcpp_action::GoalUUID & uuid,
    const char * result) noexcept;

  /// Number of goals with a currently-open span. For tests/introspection.
  std::size_t active_count() const noexcept;

private:
  mutable std::mutex mutex_;
  std::unordered_map<std::string, ::robotops::DetachedSpan> active_;
};

}  // namespace robotops::trace::ros2_control

#endif  // ROBOTOPS_TRACE_ROS2_CONTROL__FOLLOW_JOINT_TRAJECTORY_TRACER_HPP_
