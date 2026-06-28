# robotops_trace_ros2_control

**Issue:** [ROB-425](https://linear.app/robotops/issue/ROB-425) &nbsp;•&nbsp; **Status:** 🟢 implemented (v0.2.0)

RobotOps Trace integration for **ros2_control** — RT-safe instrumentation of the
controller / **FollowJointTrajectory** action-server boundary. One span per goal,
from accept to terminal result, carrying the canonical goal UUID so the
correlation agent ([ROB-427](https://linear.app/robotops/issue/ROB-427)) stitches
the controller hop **under the action client**. Built on the
[`robotops_trace_cpp`](https://linear.app/robotops/issue/ROB-443) SDK core (>=0.3.0,
for the detached-span API); attribute keys come from the shared
`robotops_trace_semconv` dictionary.

## What it covers

A **detached SERVER span** per FollowJointTrajectory goal, opened on
**goal-accept** and closed on **result** (succeeded / aborted / canceled). On it:

| attribute | source | semconv key |
|---|---|---|
| goal UUID (RFC-4122 8-4-4-4-12 lowercase) | `goal_handle->get_goal_id()` | `robot.action.goal_id` |
| action name | the controller's FJT action | `robot.action.name` |
| terminal outcome (`succeeded`/`aborted`/`canceled`) | the goal terminal | `robot.action.result` |
| joint names (comma-joined¹) | `goal.trajectory.joint_names` | `robot.joint.name` |
| joint count | `goal.trajectory.joint_names.size()` | `robot.joint.count` |
| trajectory point count | `goal.trajectory.points.size()` | `robot.trajectory.point_count` |

Span **kind = SERVER** (it is the action server side). Result → span status:
`succeeded` ⇒ Ok, `aborted` ⇒ Error, `canceled` ⇒ Unset.

¹ semconv types `robot.joint.name` as `str[]`; the SDK core 0.3.0
`AttributeValue` is scalar-only ("arrays come later"), so until the core ships
array attributes the joint names are emitted as a single comma-joined string.
`robot.joint.count` is the unambiguous queryable cardinality.

### The goal-UUID stitching contract (ROB-427)

The goal UUID is formatted **byte-for-byte identically** to the rclcpp/rclpy
action *client* (`robotops_trace_rclcpp`'s `goal_id_to_string`). Both client and
server observe the same 16 goal bytes, so both emit the same canonical string —
that identity is what lets the agent join the controller's server span under the
action client's span across the process boundary. Do not change the formatting.

## RT-safety (hard guarantee)

**Nothing in this integration ever runs in the real-time `update()` control
loop** — no span op, no allocation, no locking, no logging. All instrumentation
runs at the action-server boundary on the executor / non-RT monitor thread. The
terminal outcome is *decided* in `update()` the stock ros2_control way (an
RT-safe `RealtimeServerGoalHandle` flag write into a lock-free buffer); the span
is *closed* later, on the non-RT thread.

This is why the span is a **detached span** ([ROB-443](https://linear.app/robotops/issue/ROB-443)):
`robotops::start_detached_span()` mints a span **without** touching any thread's
current-context, so it can be held from accept to result across the controller's
async execution (RT loop + non-RT monitor) and ended on whatever thread observes
the terminal — with an explicit parent, never the wrong thread-local context.

**Zero-robot-impact:** every helper method is `noexcept` and wraps its body in a
catch-all; the underlying detached-span ops are noexcept and degrade to cheap
no-ops when the SDK is disabled/uninitialised. A tracing fault can never perturb
the controller's goal handling.

## No-fork delivery: the carry-patch model

A stock `joint_trajectory_controller` creates its action server internally, so
there is no public hook. Rather than fork it, this package ships **both**:

1. **A reusable RT-safe helper** — `FollowJointTrajectoryTracer` (this package's
   public header). The tested, framework artifact.
2. **A carried patch** — `patches/0001-...patch` wires the helper into stock
   `joint_trajectory_controller` at the accept/result boundary. See
   [`patches/README.md`](patches/README.md). CI applies it against stock JTC to
   prove it still applies + compiles per distro. Minimized and upstreamed over
   time (spec §3.4), same model as the MoveIt async patch.

**Customers either** build our patched `joint_trajectory_controller`
(carry-patch) **or**, for a custom controller, call the helper directly.

### Adopt for a custom controller

```cpp
#include "robotops_trace_ros2_control/robotops_trace_ros2_control.hpp"

class MyController : public controller_interface::ControllerInterface {
  robotops::trace::ros2_control::FollowJointTrajectoryTracer traj_tracer_;

  // handle_accepted (executor / NON-RT):
  void handle_accepted(std::shared_ptr<GoalHandle> gh) {
    traj_tracer_.on_goal_accepted(
      gh->get_goal_id(), *gh->get_goal(), "/my_controller/follow_joint_trajectory");
    // ... dispatch execution; do NOT call the tracer from update() ...
  }

  // when the goal terminates, on the NON-RT thread:
  //   traj_tracer_.on_result(uuid, robotops::trace::semconv::action_result::kSucceeded);
};
```

`on_result` is idempotent and keyed by goal UUID, so it is safe to call it from a
polling monitor timer that may fire repeatedly after the terminal.

## Distributes to

- **apt** as `ros-<distro>-robotops-trace-ros2-control` (one deb per ROS distro).

## What's deferred

- **On-hardware end-to-end (ROB-435):** a full `controller_manager` + hardware
  run with a live FollowJointTrajectory goal, asserting the controller span
  nests under a real rclcpp/rclpy action client. Also the precise RT-decided
  succeeded-vs-aborted mapping for the natural-completion path (see
  `patches/README.md`, "Known fidelity gap").
