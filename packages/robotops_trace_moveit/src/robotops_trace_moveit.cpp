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

#include "robotops_trace_moveit/robotops_trace_moveit.hpp"

#include <utility>

namespace robotops::trace::moveit
{

const char * version() noexcept
{
  return "0.2.0";
}

// ---------------------------------------------------------------------------
// ExecutionScope — restore-on-execute RAII.
// ---------------------------------------------------------------------------

TrajectoryExecutionTracer::ExecutionScope::ExecutionScope(
  const ::robotops::Context & captured, const char * name,
  const TrajectoryInfo & info) noexcept
{
  try {
    // Install the captured context as current on THIS (execution) thread so the
    // execute span — and anything opened beneath it — nests under move_action.
    // Skipped when nothing was captured (no parent / disabled SDK); the execute
    // span then opens as a root, still a useful record.
    if (captured.valid()) {
      ctx_.emplace(captured);
    }

    // Open the execute span. parent == null => the SDK uses the current
    // thread-local context, which is the captured (move_action) context we just
    // installed. The SpanGuard pushes itself as current for the scope, so the
    // FollowJointTrajectory action client opened later in executePart() nests
    // under it.
    ::robotops::SpanOptions opts;
    opts.kind = ::robotops::SpanKind::Internal;
    span_.emplace(name, opts);

    ::robotops::Span s = span_->span();
    s.set_attribute(keys::kRobotComponentName, "trajectory_execution_manager");
    if (info.point_count >= 0) {
      s.set_attribute(keys::kRobotTrajectoryPointCount, info.point_count);
    }
    if (info.joint_count >= 0) {
      s.set_attribute(keys::kRobotJointCount, info.joint_count);
    }
    if (!info.joint_names.empty()) {
      s.set_attribute(keys::kRobotJointName, info.joint_names);
    }
  } catch (...) {
    // Swallow: tracing must be invisible to the robot. If the span failed to open
    // the optionals stay empty and the scope is inert.
  }
}

::robotops::Span TrajectoryExecutionTracer::ExecutionScope::span() const noexcept
{
  if (span_.has_value()) {
    return span_->span();
  }
  return ::robotops::Span{};
}

bool TrajectoryExecutionTracer::ExecutionScope::active() const noexcept
{
  return span_.has_value() && span_->span().valid();
}

// ---------------------------------------------------------------------------
// TrajectoryExecutionTracer — capture-on-enqueue / restore-on-execute.
// ---------------------------------------------------------------------------

void TrajectoryExecutionTracer::on_enqueue(Key key) noexcept
{
  // ZERO-ROBOT-IMPACT: a tracing fault must never perturb push(). capture_context
  // is noexcept; this catch-all additionally contains map-allocation throws.
  try {
    ::robotops::Context ctx = ::robotops::capture_context();
    if (!ctx.valid()) {
      return;   // nothing active to carry (no move_action span / disabled SDK)
    }
    std::lock_guard<std::mutex> lock(mutex_);
    captured_.insert_or_assign(key, std::move(ctx));
  } catch (...) {
    // Swallow.
  }
}

TrajectoryExecutionTracer::ExecutionScope TrajectoryExecutionTracer::on_execute(
  Key key, const TrajectoryInfo & info, const char * span_name) noexcept
{
  ::robotops::Context captured;
  try {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = captured_.find(key);
    if (it != captured_.end()) {
      captured = it->second;
      captured_.erase(it);
    }
  } catch (...) {
    // Swallow; fall through with an invalid context (execute span opens as root).
  }
  // Returned as a prvalue: guaranteed copy elision (C++17), no move required.
  return ExecutionScope(captured, span_name, info);
}

void TrajectoryExecutionTracer::discard(Key key) noexcept
{
  try {
    std::lock_guard<std::mutex> lock(mutex_);
    captured_.erase(key);
  } catch (...) {
    // Swallow.
  }
}

std::size_t TrajectoryExecutionTracer::pending_count() const noexcept
{
  std::lock_guard<std::mutex> lock(mutex_);
  return captured_.size();
}

}  // namespace robotops::trace::moveit
