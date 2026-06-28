# MoveIt async trace patch (★ carry-patch)

**Issue:** [ROB-426](https://linear.app/robotops/issue/ROB-426)

MoveIt's `TrajectoryExecutionManager` (TEM) **queues** a trajectory on one thread
(`push()` / `pushAndExecute()`, called from the MoveGroup `move_action` execute
callback) and **executes** it on a *different* thread (the execution thread spun
up by `execute()`, which runs `executeThread()` → `executePart()`). The SDK's
thread-local current-context lives on the *enqueue* thread, so by the time the
execution thread sends the trajectory to the controller (the
`FollowJointTrajectory` action client), the trace context is gone — the
controller hop becomes a separate trace **root** instead of nesting under
`move_action`. This is the **one residual MoveIt async case that has no public
hook**: there is no callback, no signal, no virtual seam at the TEM
enqueue/execute boundary to hang instrumentation off without touching the source.

To close it **without a fork**, this package carries a small patch that wires the
capture/restore helper (`robotops::trace::moveit::TrajectoryExecutionTracer`,
shipped by this package) into stock `moveit_ros_planning`'s TEM.

Same **carry-patch-now → upstream-later** strategy as the rest of the
integration suite (spec §3.4). Per spec this MoveIt TEM patch is the canonical
example of the carried-fork exception: it is minimized and pushed upstream over
time (a proper tracing-hook in TEM — e.g. virtual `onTrajectoryEnqueued()` /
`onTrajectoryExecuting()` seams, or first-class context fields on
`TrajectoryExecutionContext` — so downstreams never need to patch). CI applies it
against stock MoveIt per distro to prove it still applies + compiles cleanly.

## Files (per-distro, ROB-449)

```
patches/
  jazzy/0001-trajectory-execution-manager-context-capture.patch   # moveit2 tag 2.12.4
  humble/0001-trajectory-execution-manager-context-capture.patch  # moveit2 tag 2.5.9
  apply.sh   # selects patches/${ROS_DISTRO}/... and apply-checks against a stock checkout
```

Each variant is pinned to the immutable upstream moveit2 tag the apt deb for that
distro is built from (jazzy → `2.12.4`, humble → `2.5.9`).

### Per-distro structural differences (ROB-449)

Both variants wire the **same** `TrajectoryExecutionTracer` helper at the same
three boundaries (`push` / `executePart` / `clear`). Humble's TEM, however,
diverges structurally from jazzy's — this is a real re-base, not a byte-copy:

- **Public header extension.** jazzy: `trajectory_execution_manager.hpp`; humble
  (moveit2 2.5.9): `trajectory_execution_manager.h` (and the surrounding moveit
  includes are `.h`, not `.hpp`). The helper include is `.hpp` in both.
- **CMake.** humble's `trajectory_execution_manager/CMakeLists.txt` links via
  `ament_target_dependencies(${MOVEIT_LIB_NAME} ...)` (so the variant appends
  `robotops_trace_moveit` to that list); the planning-level CMake uses
  `THIS_PACKAGE_INCLUDE_DEPENDS` + a `find_package(robotops_trace_moveit REQUIRED)`.
- **Member anchor.** on humble, `trajectories_` is followed by
  `continuous_execution_queue_`; the tracer member is inserted right after
  `trajectories_` in both.

The C++ hook bodies (`on_enqueue` / `on_execute` / `discard`) are identical across
distros — the helper API is framework-version-agnostic.

## What the patch does (and does NOT do)

Originally authored against **moveit2 2.12.4 (jazzy)**; the **humble** variant is
the same wiring re-based onto **moveit2 2.5.9**. It is **+63 / −1 lines** across
five files, all of `moveit_ros/planning`:

- **`trajectory_execution_manager.hpp`** — `#include` the helper and add one
  member: `robotops::trace::moveit::TrajectoryExecutionTracer
  robotops_traj_tracer_;`.
- **`trajectory_execution_manager.cpp`**:
  - **capture-on-enqueue** — at the end of `push()` (caller thread), after the
    trajectory is appended to the queue:
    `robotops_traj_tracer_.on_enqueue(context);` snapshots the active
    `move_action` context keyed by the queued `TrajectoryExecutionContext*`.
  - **restore-on-execute** — at the **top of `executePart()`** (execution
    thread): compute the trajectory/joint counts, then
    `auto robotops_exec_scope = robotops_traj_tracer_.on_execute(&context, info);`
    re-establishes the captured context and opens the `moveit.execute` span. The
    RAII scope lives to the end of `executePart()`, covering `sendTrajectory()`
    (which opens the controller action client → nests under `moveit.execute` →
    nests under `move_action`) **and** the `waitForExecution()` block.
  - **discard-on-clear** — in `clear()`: `robotops_traj_tracer_.discard(trajectory);`
    drops the capture for a pushed-but-never-executed trajectory so captures do
    not accumulate.
- **`CMakeLists.txt`** (both the package top-level and the
  `trajectory_execution_manager` sub-directory) and **`package.xml`** — the
  `find_package` / `ament_target_dependencies` / `<depend>` lines that link the
  helper into `moveit_trajectory_execution_manager`.

**It does NOT** touch any controller, any RT path, or any other MoveIt package.
The execute-span attribute computation (point/joint counts) runs on the TEM
*execution thread*, which is **not** the controller's real-time loop (that lives
in the controller process, reached over the `FollowJointTrajectory` action) — so
there is no real-time impact. The whole hook is `noexcept` + catch-all inside the
helper and degrades to a no-op when the SDK is disabled, so a tracing fault can
never perturb MoveIt's execution.

## How nesting works across the boundary

```
move_action (SERVER, on the move_group action server)         [enqueue thread]
└─ moveit.execute (INTERNAL, opened in executePart)           [execution thread]
   └─ <controller>/follow_joint_trajectory (CLIENT/SERVER)    [controller hop]
```

`on_enqueue` captures `move_action`'s context as a value snapshot;
`on_execute` installs it with `robotops::ScopedContext` on the execution thread
and opens `moveit.execute` on top of it; the action client opened inside
`sendTrajectory()` inherits the (now-current) `moveit.execute` context. Net: the
controller hop is no longer a separate root — it stitches under `move_action`.

## Applying / adopting

Customers have two options:

1. **Use our build of `moveit_ros_planning`** (this patch applied) — the
   carry-patch model. Run `apply.sh` against a stock `moveit2` checkout pinned to
   the per-distro release (jazzy → **2.12.4**, humble → **2.5.9**) before building.
2. **Custom integration** — don't patch anything; call the helper directly from
   your own code if you drive TEM yourself: `on_enqueue(key)` where you push,
   `on_execute(key, info)` at the top of your execution step. The patch is just
   the reference wiring of that same helper into stock TEM.

```bash
# against a stock moveit2 checkout (distro from $ROS_DISTRO, or arg 2):
ROS_DISTRO=humble ./apply.sh /path/to/moveit2
./apply.sh /path/to/moveit2 jazzy
# or directly:
git -C /path/to/moveit2 apply humble/0001-trajectory-execution-manager-context-capture.patch
```
