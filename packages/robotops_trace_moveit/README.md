# robotops_trace_moveit

**Issue:** [ROB-426](https://linear.app/robotops/issue/ROB-426) &nbsp;•&nbsp; **Status:** 🟢 real integration

RobotOps Trace integration for **MoveIt** — the ★ async-boundary case. It carries
the trace context across MoveIt's `TrajectoryExecutionManager` (TEM) internal
queue so that `move_action → moveit.execute → FollowJointTrajectory` nests
**end to end**, instead of the controller hop breaking off into a separate trace.
Built on the `robotops_trace_cpp` SDK core (≥ 0.3.0) and the shared
`robotops_trace_semconv` keys.

## The ★ problem: a queue that loses the trace context

MoveIt executes a planned trajectory through TEM, which is **asynchronous across
a thread boundary**:

- **Enqueue thread** — the MoveGroup `move_action` execute callback calls
  `TrajectoryExecutionManager::push()` (then `execute()`). This is where the
  `move_action` span is live on the thread-local current context.
- **Execution thread** — `execute()` spins up a *separate* thread that runs
  `executeThread()` → `executePart()`, which sends each trajectory to its
  controller via the `FollowJointTrajectory` action client.

The SDK's current-context is **thread-local**, so it does not cross that queue.
By the time `executePart()` sends the trajectory, the `move_action` context is
gone — the controller's action-client span has no parent and starts a **brand-new
trace root**. The end-to-end story (plan → execute → controller → hardware) is
split into two disconnected traces. This is the **one residual MoveIt async case
that has no public hook** to fix cleanly — there is no callback or virtual seam at
the TEM enqueue/execute boundary.

## The fix: capture-on-enqueue / restore-on-execute

This package closes the gap with the SDK core 0.3.0 async-context API
(`robotops::capture_context()` / `robotops::ScopedContext`):

```
move_action (SERVER)                                          [enqueue thread]
└─ moveit.execute (INTERNAL)        ← opened in executePart   [execution thread]
   └─ <controller>/follow_joint_trajectory                    [controller hop]
```

1. **Capture (enqueue):** when a trajectory is enqueued, snapshot the active
   trace context (`capture_context()`, a copyable value) and store it keyed by the
   queued trajectory's identity.
2. **Restore (execute):** on the execution thread, before sending the trajectory,
   re-install that snapshot as the current context (`ScopedContext`) and open the
   `moveit.execute` span on top of it. Because the captured context is current,
   `moveit.execute` nests under `move_action`; because the span then becomes
   current for the scope, the `FollowJointTrajectory` action client opened in
   `sendTrajectory()` nests under `moveit.execute`. The context survives the
   queue → execution-thread hop, and the controller hop stitches back under
   `move_action`.

`executePart()` is one synchronous scope (it sends **and** blocks in
`waitForExecution()` before returning), so an RAII span over that scope correctly
bounds the execution — which is why this side uses `ScopedContext` + an RAII
`SpanGuard` rather than the detached-span primitive the ros2_control SERVER side
uses (there, accept and result land on different stacks).

## What's in the package

- **`include/robotops_trace_moveit/trajectory_execution_tracer.hpp`** — the
  reusable **`TrajectoryExecutionTracer`** helper: `on_enqueue(key)` (capture),
  `on_execute(key, info)` → RAII `ExecutionScope` (restore + open span),
  `discard(key)`, `pending_count()`. It is **MoveIt-agnostic** — it deals only in
  trace contexts plus a small `TrajectoryInfo` POD, so it builds and is
  unit-tested **without a MoveIt install**.
- **`patches/`** — the carried patch wiring the helper into stock
  `moveit_ros_planning`'s TEM (`0001-…context-capture.patch` + `apply.sh`). See
  [`patches/README.md`](patches/README.md). **Carry-patch now → upstream a proper
  TEM tracing hook later** (spec §3.4); this MoveIt TEM patch is the canonical
  carried-fork exception.
- **`include/robotops_trace_moveit/semantic_conventions.hpp`** — the span keys,
  all re-exported from `robotops_trace_semconv` (no MoveIt-local keys).

## Span emitted

`moveit.execute` — kind **INTERNAL**, parented to the captured `move_action`
context. Attributes (all from `robotops_trace_semconv`):

| key | meaning |
| --- | --- |
| `robot.trajectory.point_count` | total trajectory points (summed over per-controller parts) |
| `robot.joint.count` | number of joints actuated |
| `robot.joint.name` | joint names (comma-joined; semconv types it `str[]`, core 0.3.0 attrs are scalar-only) |
| `robot.component.name` | `trajectory_execution_manager` |

The MoveGroup goal UUID is **not** available at the TEM layer (it lives up in the
`move_action` server, not on the trajectory), so `robot.action.*` is emitted by
the rclcpp/rclpy action layer, not here; nesting under `move_action` provides the
linkage.

## Zero robot impact

Every helper method is `noexcept` + catch-all wrapped; the underlying
capture/attach/span ops are themselves `noexcept` and degrade to cheap no-ops when
the SDK is disabled / uninitialised. Nothing blocks the execution thread beyond a
value-copy capture and a span open, and **nothing runs in the controller's
real-time loop** (that lives in the controller process, reached over the
`FollowJointTrajectory` action). A tracing fault can never perturb MoveIt's
execution. The kill switch (`ROBOTOPS_TRACE_ENABLED=0`) makes the whole path inert.

## Adopt for a custom integration

If you drive TEM yourself (or any analogous enqueue/execute queue), use the helper
directly — no patch needed:

```cpp
#include <robotops_trace_moveit/robotops_trace_moveit.hpp>

robotops::trace::moveit::TrajectoryExecutionTracer tracer;   // hold for the lifetime

// where you enqueue (caller thread, under move_action):
tracer.on_enqueue(/*key=*/queued_trajectory_ptr);

// at the top of your execution step (execution thread):
robotops::trace::moveit::TrajectoryInfo info;
info.point_count = /* ... */; info.joint_count = /* ... */; info.joint_names = "...";
auto scope = tracer.on_execute(/*key=*/queued_trajectory_ptr, info);
// ... send the trajectory; the controller action client nests under moveit.execute ...
// scope closes at end of this block.
```

Otherwise, use our patched build of `moveit_ros_planning` (run `patches/apply.sh`
against a stock `moveit2` checkout before building).

## Verification

In `ros:jazzy` Docker (core 0.3.0 from `apt.development`, semconv from source,
MoveIt 2.12.4 from apt):

- **Helper test (the real proof):** a gtest simulates the async boundary across
  two real threads — captures a context under a `move_action` span on thread A,
  hands it across to thread B, restores it + opens `moveit.execute` with a child
  `follow_joint_trajectory` span, and asserts via the core `InMemorySpanExporter`
  that the chain nests `move_action → moveit.execute → follow_joint_trajectory`
  (one trace id, exact parent_span_id links) — i.e. the context survived the
  thread/queue hop. Plus per-trajectory keying, `discard`, no-context-root, and
  disabled-SDK no-op. **5 passed**, ament lints green.
- **Carried patch:** **applies cleanly** (`git apply --check` exit 0) against stock
  moveit2 2.12.4, and the patched `trajectory_execution_manager.cpp`
  **compiles + links** against the apt MoveIt underlay (the patched object
  references the helper's `on_enqueue`/`on_execute`/`discard`).
- **Deferred to on-hardware (ROB-435):** the full live `move_group` → controller →
  hardware end-to-end run (real plan, real controller span nesting under a real
  `move_action`) needs the robot stack and is tracked there.

## Distributes to

- **apt** as `ros-<distro>-robotops-trace-moveit` (one deb per ROS distro).
