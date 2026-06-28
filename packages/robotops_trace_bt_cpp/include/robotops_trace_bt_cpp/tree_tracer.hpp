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

#ifndef ROBOTOPS_TRACE_BT_CPP__TREE_TRACER_HPP_
#define ROBOTOPS_TRACE_BT_CPP__TREE_TRACER_HPP_

#include <cstdint>
#include <unordered_map>

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/abstract_logger.h"
#include "robotops_trace/trace.hpp"

#include "robotops_trace_bt_cpp/semantic_conventions.hpp"

/// \file tree_tracer.hpp
/// \brief `TreeTracer` — the customer entry point. One RobotOps span per BT node
/// EXECUTION, nested by the behavior-tree STRUCTURE.
///
/// MECHANISM (and why it is fork-free): BehaviorTree.CPP exposes a public,
/// supported observer seam — `BT::StatusChangeLogger` (the same hook Groot/the
/// built-in file loggers use). Its constructor walks the tree once and
/// subscribes to every node's status-change signal; each transition invokes our
/// `callback(timestamp, node, prev_status, status)`. So we instrument stock
/// BT.CPP with ZERO fork and zero changes to the user's nodes — exactly what
/// Nav2 and MoveIt Pro need, since both run upstream BehaviorTree.CPP.
///
/// SPAN MODEL — one span per node EXECUTION, not per tick. BT.CPP re-ticks the
/// whole tree at 10–100 Hz; a long-running async node is ticked on every pass.
/// Emitting a span per tick would be a flood of meaningless ~0-duration spans.
/// Instead we open ONE span when a node ENTERS execution (IDLE -> RUNNING, or
/// IDLE -> terminal for a synchronous node that completes in a single tick) and
/// close it when the node reaches a TERMINAL status (SUCCESS / FAILURE / a halt
/// to IDLE / SKIPPED). The span's duration is the node's real execution time
/// across however many ticks that took.
///
/// NESTING — by TREE STRUCTURE, not thread-local context. A long-running async
/// node is ticked across many different call stacks (and, under a
/// multi-threaded executor driving the tree, potentially different threads), so
/// the SDK's thread-local current-context is the WRONG parent source. Instead we
/// walk the tree once at attach time to build a child-UID -> parent-UID map, and
/// open each node's span with an EXPLICIT parent (`SpanOptions.parent`) set to
/// the parent node's still-open span context. This is correct because a control
/// node transitions to RUNNING *before* it ticks its children, so its span is
/// already active when each child's span opens.
///
/// ZERO-ROBOT-IMPACT: the status-change callback never throws and never blocks —
/// every SDK span operation is `noexcept`, the whole callback body is wrapped in
/// a catch-all, and a disabled/uninitialised SDK degrades every span op to a
/// cheap no-op. An exception or a misconfigured tracer must never perturb a tick.

namespace robotops::trace::bt
{

namespace detail
{

/// An open per-node span: the RAII guard that keeps it open across ticks, plus a
/// cached copy of its `SpanContext` so child nodes can reference a stable address
/// for `SpanOptions.parent`. (`std::unordered_map` guarantees element pointer
/// stability across insert/erase, so `&active_[uid].context` stays valid for the
/// span's lifetime.)
struct ActiveSpan
{
  ::robotops::SpanContext context;
  ::robotops::SpanGuard guard;
};

}  // namespace detail

/// Attach this to a tree to emit one RobotOps span per node execution, nested by
/// tree structure. Construct it AFTER building the tree; keep it alive for as
/// long as you tick the tree.
///
/// \code
///   BT::Tree tree = factory.createTree("MainTree");
///   robotops::trace::bt::TreeTracer tracer(tree);   // attaches; no fork needed
///   tree.tickWhileRunning();                        // spans emitted per node
/// \endcode
///
/// Works unchanged with Nav2's `BehaviorTreeEngine`/`BtActionServer` trees and
/// MoveIt Pro's Objective trees — both expose the parsed `BT::Tree`.
class TreeTracer : public ::BT::StatusChangeLogger
{
public:
  /// Attach to a fully-built tree. Walks the tree once to build the
  /// parent-UID map, then subscribes to every node's status changes.
  explicit TreeTracer(const ::BT::Tree & tree);

  ~TreeTracer() override = default;

  TreeTracer(const TreeTracer &) = delete;
  TreeTracer & operator=(const TreeTracer &) = delete;
  TreeTracer(TreeTracer &&) = delete;
  TreeTracer & operator=(TreeTracer &&) = delete;

  /// The StatusChangeLogger hook. Invoked by BT.CPP on every node status
  /// transition. Opens/closes per-execution spans. Never throws.
  void callback(
    ::BT::Duration timestamp,
    const ::BT::TreeNode & node,
    ::BT::NodeStatus prev_status,
    ::BT::NodeStatus status) override;

  /// StatusChangeLogger requires this; the SDK batches/exports on its own
  /// schedule, so there is nothing to flush here.
  void flush() override {}

private:
  /// Walk the tree from `root` building child-UID -> parent-UID.
  void build_parent_map(const ::BT::TreeNode * root);

  /// Open a per-execution span for `node`, parented (explicitly) to its parent
  /// node's still-open span. No-op if already open.
  void open_span(const ::BT::TreeNode & node);

  /// Close the open span for `uid`, stamping the terminal status. No-op if none.
  void close_span(std::uint16_t uid, ::BT::NodeStatus terminal_status);

  std::unordered_map<std::uint16_t, std::uint16_t> parent_uid_;
  std::unordered_map<std::uint16_t, detail::ActiveSpan> active_;
};

}  // namespace robotops::trace::bt

#endif  // ROBOTOPS_TRACE_BT_CPP__TREE_TRACER_HPP_
