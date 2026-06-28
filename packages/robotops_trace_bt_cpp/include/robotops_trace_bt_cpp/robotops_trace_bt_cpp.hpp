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

#ifndef ROBOTOPS_TRACE_BT_CPP__ROBOTOPS_TRACE_BT_CPP_HPP_
#define ROBOTOPS_TRACE_BT_CPP__ROBOTOPS_TRACE_BT_CPP_HPP_

/// \file robotops_trace_bt_cpp.hpp
/// \brief Umbrella include for the RobotOps Trace BehaviorTree.CPP integration
/// (ROB-424).
///
/// Opt-in, fork-free instrumentation for BehaviorTree.CPP — covers both Nav2 and
/// MoveIt Pro, which run upstream BT.CPP. Attach `TreeTracer` to a built tree to
/// emit one span per node EXECUTION, nested by the tree structure. Built on the
/// transport-agnostic `robotops_trace_cpp` SDK core; attribute keys come from the
/// shared `robotops_trace_semconv` dictionary where they fit.

#include "robotops_trace_bt_cpp/semantic_conventions.hpp"  // keys::*
#include "robotops_trace_bt_cpp/tree_tracer.hpp"           // TreeTracer

namespace robotops::trace::bt
{

/// Returns the package version string (matches package.xml).
const char * version() noexcept;

}  // namespace robotops::trace::bt

#endif  // ROBOTOPS_TRACE_BT_CPP__ROBOTOPS_TRACE_BT_CPP_HPP_
