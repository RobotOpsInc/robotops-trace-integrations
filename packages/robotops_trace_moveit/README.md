# robotops_trace_moveit

**Issue:** [ROB-426](https://linear.app/robotops/issue/ROB-426) &nbsp;•&nbsp; **Status:** 🟡 stub / scaffold

RobotOps Trace integration for **MoveIt** (★ async patch package).

Captures/restores trace context across MoveIt's TrajectoryExecutionManager internal async boundary. This package **carries a not-yet-upstream patch** (see `patches/`); CI builds it against stock MoveIt to prove the patch applies cleanly per distro. Carried until upstreamed (spec §3.4). Built on the `robotops_trace_cpp` SDK core. **ROB-426** fills in the real capture/restore + the patch.

## What's here now (scaffold)

- `package.xml` (Format 3, v0.1.0, Apache-2.0) declaring the dependency on
  `robotops_trace_cpp` (>=0.1.0) and `robotops_trace_semconv` (>=0.1.0) plus
  the target framework.
- A trivial `ament_cmake` `CMakeLists.txt` building/installing a placeholder
  library + stub header.

## Distributes to

- **apt** as `ros-<distro>-robotops-trace-moveit` (one deb per ROS distro).

> **Heads-up:** the build depends on `ros-<distro>-robotops-trace-cpp`, which is
> **not yet published**. Per-distro CI will fail at rosdep resolution until the
> SDK core ships. This is expected for the scaffold.
