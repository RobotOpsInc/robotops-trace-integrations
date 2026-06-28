# robotops_trace_bt_cpp

**Issue:** [ROB-424](https://linear.app/robotops/issue/ROB-424) &nbsp;•&nbsp; **Status:** 🟢 first cut

Opt-in, **fork-free** RobotOps Trace instrumentation for **BehaviorTree.CPP**.
One hook covers **both Nav2 and MoveIt Pro**, because both run upstream
BehaviorTree.CPP — Nav2's navigators (`bt_navigator`) and MoveIt Pro's Objectives
are both `BT::Tree`s. Built on the transport-agnostic
[`robotops_trace_cpp`](../../) SDK core; attribute keys come from the shared
[`robotops_trace_semconv`](../robotops_trace_semconv) dictionary where they fit.

```cpp
#include <robotops_trace_bt_cpp/robotops_trace_bt_cpp.hpp>
namespace rtb = robotops::trace::bt;

BT::Tree tree = factory.createTree("MainTree");
rtb::TreeTracer tracer(tree);   // attach AFTER building the tree — no fork
tree.tickWhileRunning();        // one span per node execution, nested by tree
```

`TreeTracer` is a small `BT::StatusChangeLogger` subclass. Construct one after
you build your tree and keep it alive while you tick. **No fork of
BehaviorTree.CPP, no changes to your nodes**, and you keep stock
BT.CPP + Nav2/MoveIt Pro.

---

## Mechanism: the public `BT::StatusChangeLogger` seam

BehaviorTree.CPP ships a public, supported observer hook —
`BT::StatusChangeLogger` (the same seam Groot and the built-in file/console
loggers use). Its constructor walks the tree once and subscribes to **every**
node's status-change signal; each transition invokes our virtual

```cpp
void callback(BT::Duration t, const BT::TreeNode& node,
              NodeStatus prev_status, NodeStatus status);
```

So we instrument the tree with **zero fork** and zero changes to user nodes —
exactly what we need for Nav2 and MoveIt Pro.

## Span model: one span per node EXECUTION (not per tick)

BehaviorTree.CPP **re-ticks the whole tree at 10–100 Hz**. A long-running async
node (e.g. `NavigateToPose`, a MoveIt Pro planning action) is ticked on every
pass and stays `RUNNING` across many ticks. Emitting a span *per tick* would be a
flood of meaningless ~0-duration spans.

Instead we emit **one span per node execution**:

| transition | meaning | action |
|---|---|---|
| `IDLE → RUNNING` | async/control node enters execution | **open** span |
| `IDLE → SUCCESS/FAILURE` | synchronous node, done in one tick | **open + close** (one span) |
| `RUNNING → SUCCESS/FAILURE` | execution finished | **close** span |
| `RUNNING → IDLE` | halted / aborted mid-execution | **close** span (`bt.status=halted`) |
| `* → SKIPPED` | skipped by a precondition | close any open span (skipped nodes get no span) |

The span's duration is the node's real execution time across however many ticks
it took.

## Nesting: by TREE STRUCTURE, not thread-local context

The obvious "nest under whatever span is active on this thread" approach is
**wrong** for behavior trees: a long-running async node is ticked across many
different call stacks (and under a multi-threaded executor driving the tree,
potentially different threads), so the SDK's thread-local current-context is not
a reliable parent.

Instead, at attach time we **walk the tree once** (`applyRecursiveVisitor` +
`ControlNode::children()` / `DecoratorNode::child()`) to build a
**child-UID → parent-UID** map. When a node's span opens we set an **explicit
parent** (`robotops::SpanOptions.parent`) to the parent node's *still-open* span
context. This is correct because **a control node transitions to `RUNNING`
before it ticks its children**, so its span is already open when each child's
span opens. The result is a span tree that mirrors the behavior tree, regardless
of which thread/tick a node runs on.

> **Implementation note (thread-local):** the SDK's only span-lifetime primitive
> is the RAII `SpanGuard`, which also pushes/pops a thread-local current-context.
> Because BT spans are held open across ticks, the thread-local current-context
> on the tick thread points at the most-recently-opened BT node while the tree is
> mid-execution. This does **not** affect BT span nesting (we always pass an
> explicit parent), but a user span opened *on the tick thread between ticks*
> could nest under a BT node. In practice that is benign (it did happen during
> that node's execution window). A non-RAII open/close primitive in the core
> would remove the side effect entirely.

## Attributes & semantic conventions

| key | value | source |
|---|---|---|
| *(span name)* | `node.name()` (the instance name) | — |
| `robot.component.name` | `node.name()` | **semconv** (`robotops_trace_semconv`) |
| `bt.node_type` | `node.registrationName()` (e.g. `Sequence`, `RetryUntilSuccessful`, a user action ID) | **bt.-local** |
| `bt.status` | terminal status: `success` / `failure` / `skipped` / `halted` | **bt.-local** |

Span status maps from the BT outcome: **`FAILURE → StatusCode::Error`**,
**`SUCCESS → StatusCode::Ok`**; a halt/skip leaves it `Unset`.

`bt.node_type` / `bt.status` are **not** in semconv v0 — the dictionary only
promotes a key once it has a stable cross-framework meaning, and behavior-tree
node type/status have no portable analogue yet. They live in
[`semantic_conventions.hpp`](include/robotops_trace_bt_cpp/semantic_conventions.hpp)
with a `TODO(semconv-v1)` noting `robot.behavior.*` as the candidate namespace
(so any BT-shaped planner — BT.CPP, py_trees, FlexBE — would share keys). This
package does **not** edit semconv; that promotion is a reviewed dictionary change.

## Zero-robot-impact

The status-change callback **never throws and never blocks**: every SDK span
operation is `noexcept`, the whole callback body is wrapped in a catch-all, and a
disabled/uninitialised SDK degrades every span op to a cheap no-op. A tracing
fault must never perturb a tick. (Verified: a tree ticked with the SDK
kill-switched still returns `SUCCESS` and emits no spans.)

## Limitations

- **Opt-in per tree:** you must construct a `TreeTracer` after building the tree.
  Fully-automatic process-wide attach is deferred to the auto-init layer
  (ROB-421).
- **Thread-local side effect** while spans are open (see note above).
- **Subtrees:** the parent map descends through `SubTree` nodes, so a subtree's
  root nests under the `SubTree` node that invoked it. Spans across a subtree
  boundary share the one in-process trace.
- `bt.node_type` / `bt.status` pending promotion to semconv v1 `robot.behavior.*`.

## Building & testing

ROS 2 is not on the host — build in Docker. This package builds against the SDK
core from the **development apt channel** (`ros-<distro>-robotops-trace-cpp`),
with `robotops_trace_semconv` built from source and `behaviortree_cpp` from apt,
in one combined colcon workspace:

```sh
colcon build --packages-up-to robotops_trace_bt_cpp
colcon test  --packages-select  robotops_trace_bt_cpp
```

The gtest suite ([`test/`](test/)) builds a `Sequence` of two actions (one an
async `StatefulActionNode` that returns `RUNNING` then `SUCCESS`), attaches a
`TreeTracer`, ticks to completion, and asserts via the core's
`InMemorySpanExporter`: (a) exactly one span per executed node — no per-tick
flood; (b) both leaves' `parent_span_id` == the Sequence's `span_id`
(tree-structured nesting), with the async leaf nesting correctly even though it
lives across multiple ticks; (c) a `FAILURE` node yields a `StatusCode::Error`
span; and (d) a disabled SDK is a no-op that does not perturb the tick.

## Distributes to

- **apt** as `ros-<distro>-robotops-trace-bt-cpp` (one deb per ROS distro).
