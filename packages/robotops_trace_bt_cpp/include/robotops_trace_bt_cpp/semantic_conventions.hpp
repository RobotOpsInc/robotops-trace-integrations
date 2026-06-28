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

#ifndef ROBOTOPS_TRACE_BT_CPP__SEMANTIC_CONVENTIONS_HPP_
#define ROBOTOPS_TRACE_BT_CPP__SEMANTIC_CONVENTIONS_HPP_

/// \file semantic_conventions.hpp
/// \brief Span-attribute keys used by the BehaviorTree.CPP integration.
///
/// The authoritative, cross-framework key registry is `robotops_trace_semconv`
/// (ROB-430). Keys that have a portable concept-level meaning are RE-EXPORTED
/// (not redefined) from that header so the emitted attribute strings are
/// byte-identical to every other RobotOps integration and to the ROSQL/agent
/// vocabulary.
///
/// The BT-specific keys below (`bt.node_type`, `bt.status`) describe a
/// BehaviorTree.CPP node's registration type and its terminal tick status.
/// Neither concept exists in semconv v0 (the dictionary is intentionally minimal
/// and only promotes a key once it has a stable cross-framework meaning), so they
/// stay local here.
///
/// TODO(semconv-v1): a future semconv minor should promote these under a portable
/// `robot.behavior.*` namespace (e.g. `robot.behavior.node_type`,
/// `robot.behavior.status`) so any behavior-tree-shaped planner (BT.CPP, py_trees,
/// FlexBE, ...) emits the same keys. Do NOT edit semconv from this package; that
/// promotion is a deliberate, reviewed dictionary change.

#include <robotops_trace_semconv/semconv.hpp>

namespace robotops::trace::bt::keys
{

namespace semconv = ::robotops::trace::semconv;

// --- keys sourced from the authoritative semconv dictionary (ROB-430) -------

/// Logical component/node name — the BT node's instance name. The span name is
/// also the node name; this records it as a queryable attribute. [str]
inline constexpr const char * kRobotComponentName = semconv::kRobotComponentName;

// --- BT-local keys NOT (yet) in semconv v0 (see file header TODO) -----------

/// The node's BehaviorTree.CPP registration type, i.e. `registrationName()`
/// (e.g. "Sequence", "RetryUntilSuccessful", or a user action ID). [str]
inline constexpr const char * kBtNodeType = "bt.node_type";

/// The node's terminal tick status: "success" | "failure" | "skipped" |
/// "halted". (RUNNING is the open state, never a terminal value.) [str]
inline constexpr const char * kBtStatus = "bt.status";

}  // namespace robotops::trace::bt::keys

#endif  // ROBOTOPS_TRACE_BT_CPP__SEMANTIC_CONVENTIONS_HPP_
