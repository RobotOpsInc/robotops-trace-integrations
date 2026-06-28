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

#ifndef ROBOTOPS_TRACE_MOVEIT__ROBOTOPS_TRACE_MOVEIT_HPP_
#define ROBOTOPS_TRACE_MOVEIT__ROBOTOPS_TRACE_MOVEIT_HPP_

/// \file robotops_trace_moveit.hpp
/// \brief Umbrella include for the RobotOps Trace MoveIt integration (ROB-426).
///
/// Closes the ★ residual MoveIt async case: `TrajectoryExecutionManager` queues a
/// trajectory on one thread and executes it on another, so without a hook the
/// trace context is lost across the queue and the controller hop becomes a
/// separate trace root. `TrajectoryExecutionTracer` captures the active context on
/// enqueue and restores it on execute, so
/// `move_action -> moveit.execute -> FollowJointTrajectory` nests end to end.
///
/// Built on the transport-agnostic `robotops_trace_cpp` SDK core (>=0.3.0, for the
/// `capture_context()` / `ScopedContext` async-context API); attribute keys come
/// from the shared `robotops_trace_semconv` dictionary. The integration ships as a
/// reusable helper plus a carried patch wiring it into stock
/// `moveit_ros_planning` (see `patches/`).

#include "robotops_trace_moveit/semantic_conventions.hpp"           // keys::*
#include "robotops_trace_moveit/trajectory_execution_tracer.hpp"    // TrajectoryExecutionTracer

namespace robotops::trace::moveit
{

/// Returns the package version string (matches package.xml).
const char * version() noexcept;

}  // namespace robotops::trace::moveit

#endif  // ROBOTOPS_TRACE_MOVEIT__ROBOTOPS_TRACE_MOVEIT_HPP_
