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

#include "robotops_trace_ros2_control/robotops_trace_ros2_control.hpp"

#include <cstdint>
#include <cstring>
#include <string>
#include <utility>
#include <vector>

#include "robotops_trace_ros2_control/identifiers.hpp"

namespace robotops::trace::ros2_control
{

namespace
{

/// Join the goal's joint names into a single comma-separated string. Semconv
/// types robot.joint.name as str[]; the SDK core 0.3.0 AttributeValue is
/// scalar-only, so we emit the joined form until the core ships array attributes.
std::string join_joint_names(const std::vector<std::string> & names)
{
  std::string out;
  for (std::size_t i = 0; i < names.size(); ++i) {
    if (i != 0) {
      out.push_back(',');
    }
    out += names[i];
  }
  return out;
}

/// Map the semconv action_result enum value onto the span status. `succeeded`
/// -> Ok; `aborted` -> Error; anything else (`canceled`) leaves it Unset (a
/// cancel is neither a success nor a server error).
::robotops::StatusCode result_to_status(const char * result) noexcept
{
  namespace ar = keys::action_result;
  if (result == nullptr) {
    return ::robotops::StatusCode::Unset;
  }
  if (std::strcmp(result, ar::kSucceeded) == 0) {
    return ::robotops::StatusCode::Ok;
  }
  if (std::strcmp(result, ar::kAborted) == 0) {
    return ::robotops::StatusCode::Error;
  }
  return ::robotops::StatusCode::Unset;
}

}  // namespace

const char * version() noexcept
{
  return "0.2.0";
}

void FollowJointTrajectoryTracer::on_goal_accepted(
  const ::rclcpp_action::GoalUUID & uuid,
  const Goal & goal,
  const std::string & action_name,
  const ::robotops::SpanContext * parent) noexcept
{
  // ZERO-ROBOT-IMPACT: a tracing fault must never perturb goal handling. Every
  // SDK op is already noexcept; this catch-all additionally contains anything
  // the standard library (string/map allocation) might throw.
  try {
    const std::string goal_id = goal_id_to_string(uuid);

    ::robotops::SpanOptions opts;
    opts.kind = ::robotops::SpanKind::Server;   // this is the action server side
    opts.parent = parent;                       // explicit parent, or null => root

    // Detached span: minted WITHOUT touching any thread's current-context, held
    // open across the controller's async execution, ended explicitly in
    // on_result() (ROB-443). The caller owns its lifetime.
    ::robotops::DetachedSpan span =
      ::robotops::start_detached_span(action_name + " follow_joint_trajectory", opts);

    span.set_attribute(keys::kRobotActionGoalId, goal_id);
    span.set_attribute(keys::kRobotActionName, action_name);
    span.set_attribute(
      keys::kRobotJointName, join_joint_names(goal.trajectory.joint_names));
    span.set_attribute(
      keys::kRobotJointCount,
      static_cast<std::int64_t>(goal.trajectory.joint_names.size()));
    span.set_attribute(
      keys::kRobotTrajectoryPointCount,
      static_cast<std::int64_t>(goal.trajectory.points.size()));

    std::lock_guard<std::mutex> lock(mutex_);
    // insert_or_assign: a re-accept of the same UUID (should not happen) replaces
    // and ends the stale span via DetachedSpan's destructor.
    active_.insert_or_assign(goal_id, std::move(span));
  } catch (...) {
    // Swallow: tracing must be invisible to the robot.
  }
}

void FollowJointTrajectoryTracer::on_result(
  const ::rclcpp_action::GoalUUID & uuid,
  const char * result) noexcept
{
  try {
    const std::string goal_id = goal_id_to_string(uuid);

    ::robotops::DetachedSpan span;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const auto it = active_.find(goal_id);
      if (it == active_.end()) {
        return;   // no span open for this goal (idempotent monitor-timer firing)
      }
      span = std::move(it->second);
      active_.erase(it);
    }

    if (result != nullptr) {
      span.set_attribute(keys::kRobotActionResult, result);
    }
    span.set_status(result_to_status(result));
    span.end();   // finalize + enqueue, WITHOUT touching thread-local context
  } catch (...) {
    // Swallow.
  }
}

std::size_t FollowJointTrajectoryTracer::active_count() const noexcept
{
  std::lock_guard<std::mutex> lock(mutex_);
  return active_.size();
}

}  // namespace robotops::trace::ros2_control
