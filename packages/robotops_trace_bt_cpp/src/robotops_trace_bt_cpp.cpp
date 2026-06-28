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

#include "robotops_trace_bt_cpp/robotops_trace_bt_cpp.hpp"

#include <utility>

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/control_node.h"
#include "behaviortree_cpp/decorator_node.h"

namespace robotops::trace::bt
{

namespace
{

/// Stable lowercase string for a terminal node status (the `bt.status` value).
const char * status_str(::BT::NodeStatus status) noexcept
{
  switch (status) {
    case ::BT::NodeStatus::SUCCESS: return "success";
    case ::BT::NodeStatus::FAILURE: return "failure";
    case ::BT::NodeStatus::SKIPPED: return "skipped";
    case ::BT::NodeStatus::IDLE: return "halted";   // RUNNING -> IDLE == halt
    case ::BT::NodeStatus::RUNNING: return "running";
  }
  return "unknown";
}

}  // namespace

const char * version() noexcept
{
  return "0.2.0";
}

TreeTracer::TreeTracer(const ::BT::Tree & tree)
: ::BT::StatusChangeLogger()   // deferred subscription: build state, THEN subscribe
{
  ::BT::TreeNode * root = tree.rootNode();

  // Build the structural parent map BEFORE subscribing, so the very first
  // callback can already resolve explicit parents.
  build_parent_map(root);

  // We want halts (a transition to IDLE) reported so we can close the spans of
  // nodes that were aborted mid-execution. (Default is already true in 4.x; set
  // it explicitly so the behaviour is independent of the upstream default.)
  enableTransitionToIdle(true);

  // Subscribe to every node's status changes (the StatusChangeLogger seam).
  subscribeToTreeChanges(root);
}

void TreeTracer::build_parent_map(const ::BT::TreeNode * root)
{
  if (root == nullptr) {
    return;
  }
  // Walk every node reachable from the root (this descends through SubTree nodes
  // too). A control node owns N children; a decorator owns exactly one. Record
  // each child's parent by UID so `open_span` can look up the explicit parent.
  ::BT::applyRecursiveVisitor(
    root,
    [this](const ::BT::TreeNode * node) {
      if (const auto * control = dynamic_cast<const ::BT::ControlNode *>(node)) {
        for (const auto * child : control->children()) {
          if (child != nullptr) {
            parent_uid_[child->UID()] = node->UID();
          }
        }
      } else if (const auto * decorator = dynamic_cast<const ::BT::DecoratorNode *>(node)) {
        if (const ::BT::TreeNode * child = decorator->child()) {
          parent_uid_[child->UID()] = node->UID();
        }
      }
    });
}

void TreeTracer::open_span(const ::BT::TreeNode & node)
{
  const std::uint16_t uid = node.UID();
  if (active_.find(uid) != active_.end()) {
    return;  // already open — don't double-open
  }

  // Resolve the EXPLICIT parent: the parent node's still-open span context.
  // `unordered_map` keeps element addresses stable, so this pointer remains
  // valid for as long as the parent's span is open (which it is: a control node
  // goes RUNNING before ticking its children).
  const ::robotops::SpanContext * parent_ctx = nullptr;
  const auto parent_it = parent_uid_.find(uid);
  if (parent_it != parent_uid_.end()) {
    const auto active_parent = active_.find(parent_it->second);
    if (active_parent != active_.end()) {
      parent_ctx = &active_parent->second.context;
    }
  }

  ::robotops::SpanOptions opts;
  opts.kind = ::robotops::SpanKind::Internal;
  opts.parent = parent_ctx;   // null => root (e.g. the tree's root node)

  ::robotops::SpanGuard guard(node.name(), opts);
  ::robotops::Span span = guard.span();
  span.set_attribute(keys::kRobotComponentName, node.name());
  span.set_attribute(keys::kBtNodeType, node.registrationName());

  active_.emplace(uid, detail::ActiveSpan{span.context(), std::move(guard)});
}

void TreeTracer::close_span(std::uint16_t uid, ::BT::NodeStatus terminal_status)
{
  const auto it = active_.find(uid);
  if (it == active_.end()) {
    return;  // nothing open for this node
  }

  ::robotops::Span span = it->second.guard.span();
  span.set_attribute(keys::kBtStatus, status_str(terminal_status));
  // Map the BT outcome onto the span status: FAILURE is an error, SUCCESS is ok.
  // A halt-to-IDLE or SKIPPED leaves the span status Unset (it is neither a
  // success nor an error of the node itself).
  if (terminal_status == ::BT::NodeStatus::FAILURE) {
    span.set_status(::robotops::StatusCode::Error, "behavior tree node returned FAILURE");
  } else if (terminal_status == ::BT::NodeStatus::SUCCESS) {
    span.set_status(::robotops::StatusCode::Ok);
  }

  active_.erase(it);   // destroys the guard -> closes + enqueues the span
}

void TreeTracer::callback(
  ::BT::Duration /*timestamp*/,
  const ::BT::TreeNode & node,
  ::BT::NodeStatus prev_status,
  ::BT::NodeStatus status)
{
  // ZERO-ROBOT-IMPACT: a tracing fault must never perturb a tick. Every SDK op
  // is already noexcept; this catch-all additionally contains anything the
  // standard library (map allocation) might throw.
  try {
    const bool was_idle = (prev_status == ::BT::NodeStatus::IDLE);
    const bool now_running = (status == ::BT::NodeStatus::RUNNING);
    const bool now_terminal =
      (status == ::BT::NodeStatus::SUCCESS || status == ::BT::NodeStatus::FAILURE);

    if (was_idle && now_running) {
      // Entering execution (async / control node): open the span.
      open_span(node);
    } else if (was_idle && now_terminal) {
      // Synchronous node: completes in a single tick (IDLE -> terminal). Open
      // and immediately close so it still gets exactly one execution span.
      open_span(node);
      close_span(node.UID(), status);
    } else if (now_terminal) {
      // Terminal after RUNNING: close the open span.
      close_span(node.UID(), status);
    } else {
      // A transition to IDLE (halt: aborted mid-execution) or to SKIPPED:
      // close any span that is still open for this node.
      close_span(node.UID(), status);
    }
  } catch (...) {
    // Swallow: tracing must be invisible to the robot.
  }
}

}  // namespace robotops::trace::bt
