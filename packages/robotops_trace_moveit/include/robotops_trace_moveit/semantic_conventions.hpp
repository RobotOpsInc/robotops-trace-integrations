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

#ifndef ROBOTOPS_TRACE_MOVEIT__SEMANTIC_CONVENTIONS_HPP_
#define ROBOTOPS_TRACE_MOVEIT__SEMANTIC_CONVENTIONS_HPP_

/// \file semantic_conventions.hpp
/// \brief Span-attribute keys used by the MoveIt integration.
///
/// The authoritative, cross-framework key registry is `robotops_trace_semconv`
/// (ROB-430). Every key this integration emits has a portable concept-level
/// meaning, so they are all RE-EXPORTED (not redefined) from that header — the
/// emitted attribute strings are byte-identical to every other RobotOps
/// integration and to the ROSQL/agent vocabulary. There are NO MoveIt-local keys:
/// the execute boundary is fully described by the existing trajectory + joint +
/// component keys. Do NOT add keys here; promote them in semconv (a deliberate,
/// reviewed dictionary change) instead.

#include <robotops_trace_semconv/semconv.hpp>

namespace robotops::trace::moveit::keys
{

namespace semconv = ::robotops::trace::semconv;

// --- Trajectory (from the queued trajectory's parts) ------------------------

/// Number of trajectory points (summed over the per-controller parts). [int64]
inline constexpr const char * kRobotTrajectoryPointCount =
  semconv::kRobotTrajectoryPointCount;

// --- Joint (from the queued trajectory's parts) -----------------------------

/// Joint name(s) actuated. Semconv types this as str[]; the SDK core 0.3.0
/// AttributeValue is scalar-only, so until the core ships array attributes we
/// emit the joint names as a single comma-joined string. The count below is the
/// unambiguous queryable cardinality. [str]
inline constexpr const char * kRobotJointName = semconv::kRobotJointName;
/// Number of joints actuated across the trajectory parts. [int64]
inline constexpr const char * kRobotJointCount = semconv::kRobotJointCount;

// --- Component (the producing node / subsystem) -----------------------------

/// Logical component that executed the trajectory. [str]
inline constexpr const char * kRobotComponentName = semconv::kRobotComponentName;

}  // namespace robotops::trace::moveit::keys

#endif  // ROBOTOPS_TRACE_MOVEIT__SEMANTIC_CONVENTIONS_HPP_
