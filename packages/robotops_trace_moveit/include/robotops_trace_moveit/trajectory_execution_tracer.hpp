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

#ifndef ROBOTOPS_TRACE_MOVEIT__TRAJECTORY_EXECUTION_TRACER_HPP_
#define ROBOTOPS_TRACE_MOVEIT__TRAJECTORY_EXECUTION_TRACER_HPP_

#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>

#include "robotops_trace/trace.hpp"

#include "robotops_trace_moveit/semantic_conventions.hpp"

/// \file trajectory_execution_tracer.hpp
/// \brief `TrajectoryExecutionTracer` — the capture-on-enqueue / restore-on-execute
/// utility that carries the trace context across MoveIt's
/// `TrajectoryExecutionManager` (TEM) internal async boundary.
///
/// THE ★ ASYNC PROBLEM. TEM QUEUES a trajectory on one thread (`push()` /
/// `pushAndExecute()`, called from the MoveGroup `move_action` execute callback)
/// and EXECUTES it on a DIFFERENT thread (the execution thread spun up by
/// `execute()`, which runs `executeThread()` -> `executePart()`). The SDK's
/// thread-local current-context lives on the ENQUEUE thread, so by the time the
/// execution thread sends the trajectory to the controller (the
/// `FollowJointTrajectory` action client the ros2_control / rclcpp integration
/// instruments), the trace context is GONE — the controller hop becomes a
/// separate trace ROOT instead of nesting under `move_action`. This is the one
/// residual MoveIt async case that has no public hook; it is closed by capturing
/// the context when the trajectory is enqueued and restoring it when it executes.
///
/// THE MECHANISM (core 0.3.0 capture/attach API).
///   * **Capture (enqueue thread):** `on_enqueue(key)` snapshots the active trace
///     context with `robotops::capture_context()` — a copyable value snapshot —
///     and stores it keyed by the queued trajectory's identity (`key`).
///   * **Restore (execution thread):** `on_execute(key, info)` looks the snapshot
///     up, installs it as the current context on THIS thread with
///     `robotops::ScopedContext`, and opens a `moveit.execute` span (INTERNAL) on
///     top of it. Because the captured context is current, the execute span nests
///     under `move_action`; because the execute span then becomes current for the
///     RAII scope, the `FollowJointTrajectory` action client opened inside
///     `executePart()` nests under the execute span. Net:
///     `move_action -> moveit.execute -> FollowJointTrajectory`, end to end across
///     the queue -> execution-thread hop.
///
/// WHY ScopedContext + an RAII span (not a detached span). `executePart()` is a
/// single SYNCHRONOUS scope on the execution thread: it sends the trajectory AND
/// blocks in `waitForExecution()` until the controller finishes, all before
/// returning. So an RAII span over that scope correctly bounds the execution, and
/// making the captured context current for the scope is exactly what lets the
/// action client (opened mid-scope, deep inside the controller handle) nest under
/// it. A detached span would NOT make itself current, so the action client would
/// not nest beneath it without threading an explicit parent all the way down —
/// which there is no seam for. (The detached-span primitive is the right tool for
/// the ros2_control SERVER side, where accept and result land on different stacks;
/// here the whole execution is one stack, so RAII + ScopedContext is cleaner.)
///
/// MOVEIT-AGNOSTIC BY DESIGN. This helper deals only in trace contexts + a small
/// POD (`TrajectoryInfo`); it does NOT include any MoveIt header. The carried TEM
/// patch computes the `TrajectoryInfo` from the MoveIt trajectory at the call site
/// and passes an opaque `key` (the `TrajectoryExecutionContext*`). That keeps the
/// helper library buildable and unit-testable WITHOUT a MoveIt install — the
/// async boundary is simulated directly (see the test suite).
///
/// ZERO-ROBOT-IMPACT. Every method is `noexcept` and catch-all wrapped; the
/// underlying capture/attach/span ops are themselves noexcept and degrade to
/// cheap no-ops when the SDK is disabled / uninitialised. A tracing fault can
/// never perturb MoveIt's execution, and nothing here blocks the execution thread
/// beyond the cheap capture (a value copy) and span open.
///
/// THREAD-SAFETY. The pending-capture map is guarded by a plain mutex, taken only
/// at the enqueue / execute boundaries (push and the start of executePart), never
/// in any tight loop.

namespace robotops::trace::moveit
{

/// Plain-old-data describing a queued trajectory, used to stamp the execute span.
/// Computed by the carried patch from the MoveIt `TrajectoryExecutionContext` at
/// the call site so this helper needs no MoveIt dependency. A negative count means
/// "unknown / do not emit".
struct TrajectoryInfo
{
  std::int64_t point_count{-1};   ///< robot.trajectory.point_count (summed over parts)
  std::int64_t joint_count{-1};   ///< robot.joint.count
  std::string joint_names;        ///< robot.joint.name (comma-joined; see semconv note)
};

/// Captures a trace context per queued trajectory and restores it for execution.
///
/// Hold one instance for the lifetime of the `TrajectoryExecutionManager` (the
/// carried patch makes it a member). Call `on_enqueue()` from `push()` on the
/// caller thread and `on_execute()` from the top of `executePart()` on the
/// execution thread.
class TrajectoryExecutionTracer
{
public:
  /// Opaque identity of a queued trajectory. The patch passes the queued
  /// `TrajectoryExecutionContext*`, which is stable from `push()` to
  /// `executePart()` and unique per queued trajectory.
  using Key = const void *;

  TrajectoryExecutionTracer() = default;
  ~TrajectoryExecutionTracer() = default;

  TrajectoryExecutionTracer(const TrajectoryExecutionTracer &) = delete;
  TrajectoryExecutionTracer & operator=(const TrajectoryExecutionTracer &) = delete;
  TrajectoryExecutionTracer(TrajectoryExecutionTracer &&) = delete;
  TrajectoryExecutionTracer & operator=(TrajectoryExecutionTracer &&) = delete;

  /// RAII scope returned by `on_execute()`. While alive it (a) holds the captured
  /// context current on the execution thread and (b) holds the `moveit.execute`
  /// span open and current. Destruction closes the span FIRST (member order), then
  /// pops the restored context — so spans opened within the scope (the controller
  /// action client) nest under the execute span, which nests under move_action.
  /// Non-movable; `on_execute()` returns it as a prvalue (guaranteed copy
  /// elision, C++17), so no move is required at the call site.
  class ExecutionScope
  {
public:
    ExecutionScope(const ExecutionScope &) = delete;
    ExecutionScope & operator=(const ExecutionScope &) = delete;
    ExecutionScope(ExecutionScope &&) = delete;
    ExecutionScope & operator=(ExecutionScope &&) = delete;
    ~ExecutionScope() = default;

    /// Handle to the execute span (invalid if no span was opened, e.g. disabled
    /// SDK). For tests / for stamping extra attributes from a custom integrator.
    ::robotops::Span span() const noexcept;

    /// True when a REAL execute span is open (i.e. the SDK is enabled and the span
    /// was minted). False for a disabled/uninitialised SDK, where the scope is an
    /// inert no-op even though the guard object exists.
    bool active() const noexcept;

private:
    friend class TrajectoryExecutionTracer;
    ExecutionScope(
      const ::robotops::Context & captured, const char * name,
      const TrajectoryInfo & info) noexcept;

    // Declaration order is load-bearing: ctx_ is declared first so it is destroyed
    // LAST (after span_), i.e. the execute span closes before its parent context
    // is popped.
    std::optional<::robotops::ScopedContext> ctx_;
    std::optional<::robotops::SpanGuard> span_;
  };

  /// Capture the active trace context for the queued trajectory `key`
  /// (capture-on-enqueue). Call from `push()` on the caller thread, after the
  /// trajectory has been appended to the queue. A no-op (stores nothing) when
  /// there is no active context / the SDK is disabled.
  void on_enqueue(Key key) noexcept;

  /// Restore the context captured for `key` and open the `moveit.execute` span on
  /// this (execution) thread (restore-on-execute). Call from the TOP of
  /// `executePart()`, before the trajectory is sent to the controller. Returns an
  /// RAII scope that must outlive the send + wait (i.e. bind it to a local for the
  /// rest of `executePart()`). If no context was captured for `key` (or the SDK is
  /// disabled), the returned scope is inactive and does nothing.
  ///
  /// \param key  The queued trajectory's identity (same value passed to on_enqueue).
  /// \param info Attributes for the execute span (point/joint counts, joint names).
  /// \param span_name The execute span name (default "moveit.execute").
  ExecutionScope on_execute(
    Key key, const TrajectoryInfo & info,
    const char * span_name = "moveit.execute") noexcept;

  /// Drop a captured context without executing it (e.g. a pushed-but-cleared
  /// trajectory). Call from `clear()` so abandoned captures do not accumulate.
  /// No-op if nothing is captured for `key`.
  void discard(Key key) noexcept;

  /// Number of captured-but-not-yet-executed contexts. For tests / introspection.
  std::size_t pending_count() const noexcept;

private:
  mutable std::mutex mutex_;
  std::unordered_map<Key, ::robotops::Context> captured_;
};

}  // namespace robotops::trace::moveit

#endif  // ROBOTOPS_TRACE_MOVEIT__TRAJECTORY_EXECUTION_TRACER_HPP_
