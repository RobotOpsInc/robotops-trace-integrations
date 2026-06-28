# joint_trajectory_controller trace patch (carry-patch)

**Issue:** [ROB-425](https://linear.app/robotops/issue/ROB-425)

A stock `joint_trajectory_controller` (JTC, part of `ros2_controllers`) creates
its `FollowJointTrajectory` action server internally, so there is no public seam
to hang trace hooks off without touching the source. To instrument it
**without a fork**, this package carries a small patch that wires the RT-safe
helper (`robotops::trace::ros2_control::FollowJointTrajectoryTracer`, shipped by
this package) into JTC's action-server boundary.

Same **carry-patch-now → upstream-later** strategy as the MoveIt async patch
(spec §3.4): the patch is minimized and pushed upstream over time; CI applies it
against stock JTC per distro to prove it still applies + compiles cleanly.

## Files (per-distro, ROB-449)

```
patches/
  jazzy/0001-joint-trajectory-controller-robotops-trace-boundary.patch   # ros2_controllers tag 4.40.1
  humble/0001-joint-trajectory-controller-robotops-trace-boundary.patch  # ros2_controllers tag 2.53.1
  apply.sh   # selects patches/${ROS_DISTRO}/... and apply-checks against a stock checkout
```

Each variant is pinned to the **immutable upstream tag the apt deb for that distro
is built from** (jazzy → `4.40.1`, humble → `2.53.1`), so the rebuilt controller
matches the underlay byte-for-byte and `git apply --check` fails loudly on drift.

**Pin fix (ROB-449):** the jazzy variant is now pinned to the `4.40.1` *tag*. The
original single patch claimed `4.40.1` but in fact only applied to the moving
`jazzy` *branch*, which had drifted past the tag (e.g. `preempt_active_goal` on the
branch is `setAborted` + `runNonRealtime`; on the `4.40.1` tag and on humble it is
`setCanceled`). The jazzy variant was re-based onto the tag so the claimed pin and
the patch agree.

### Per-distro structural differences

Both variants wire the **same** `FollowJointTrajectoryTracer` helper at the same
four boundaries. They differ only where the upstream source differs:

- **CMake linkage.** jazzy (`4.40.1`) links the helper via the modern
  `target_link_libraries(... robotops_trace_ros2_control::robotops_trace_ros2_control)`
  block; humble (`2.53.1`) has no such block — it uses
  `ament_target_dependencies(... ${THIS_PACKAGE_INCLUDE_DEPENDS})`, so the variant
  adds `robotops_trace_ros2_control` to `THIS_PACKAGE_INCLUDE_DEPENDS` (which both
  `find_package`s it and links it). The C++ wiring is otherwise byte-identical.

## What the patch does (and does NOT do)

It adds, to `joint_trajectory_controller`:

- a `FollowJointTrajectoryTracer` member;
- `on_goal_accepted(...)` at the **top of `goal_accepted_callback`** (executor /
  NON-RT thread) — opens one detached **SERVER** span carrying the canonical goal
  UUID + the joint/trajectory semconv;
- `on_result(...)` at the three terminal boundaries that run on the **NON-RT**
  thread:
  - `goal_cancelled_callback` → `canceled`,
  - `preempt_active_goal` → `aborted` (preemption),
  - the **action-monitor timer** (the existing non-RT `runNonRealtime` timer) →
    `succeeded` once the goal handle goes inactive (the natural-completion path);
- the `find_package` + `target_link_libraries` lines to link the helper.

**RT-safety — the hard guarantee:** the patch adds **zero** lines to the
real-time `update()` control loop. No span op, no allocation, no lock, no log
runs on the RT path. The terminal outcome is still *decided* inside `update()`
exactly the stock ros2_control way (an RT-safe `RealtimeServerGoalHandle` flag
write into a lock-free buffer); the span is *closed* later, on the non-RT
monitor thread. Verify with: `grep -n "update(" the patched .cpp` — the
`update()` body is unchanged.

**Known fidelity gap (deferred to on-hardware ROB-435):** a
tolerance-violation abort decided inside `update()` (with no cancel/preempt) is
currently reported by the monitor-timer hook as `succeeded`, because the precise
RT-decided outcome is not yet threaded out to the non-RT thread. Cancel and
preempt are authoritative (they close the span first; `on_result` is
idempotent). Threading the exact success/abort outcome across the RT boundary,
and the full controller_manager + hardware end-to-end run, are tracked in
ROB-435.

## Applying / adopting

Customers have two options:

1. **Use our build of `joint_trajectory_controller`** (this patch applied) — the
   carry-patch model. Run `apply.sh` against a stock `ros2_controllers` checkout
   pinned to the per-distro release (jazzy → **4.40.1**, humble → **2.53.1**)
   before building.
2. **Custom controller** — don't patch anything; call the helper directly from
   your own action callbacks (see the package README, "Adopt for a custom
   controller"). The patch is just the reference wiring of that same helper.

```bash
# against a stock ros2_controllers checkout (distro from $ROS_DISTRO, or arg 2):
ROS_DISTRO=humble ./apply.sh /path/to/ros2_controllers
./apply.sh /path/to/ros2_controllers jazzy
# or directly:
git -C /path/to/ros2_controllers apply humble/0001-joint-trajectory-controller-robotops-trace-boundary.patch
```
