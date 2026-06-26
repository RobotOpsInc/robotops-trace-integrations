# robotops_trace_bt_cpp

**Issue:** [ROB-424](https://linear.app/robotops/issue/ROB-424) &nbsp;•&nbsp; **Status:** 🟡 stub / scaffold

RobotOps Trace integration for **BehaviorTree.CPP**.

Instruments BT tick callbacks — covers both Nav2 and MoveIt Pro behavior trees. Built on the `robotops_trace_cpp` SDK core. **ROB-424** fills in the real tick-callback hooks.

## What's here now (scaffold)

- `package.xml` (Format 3, v0.1.0, Apache-2.0) declaring the dependency on
  `robotops_trace_cpp` (>=0.1.0) and `robotops_trace_semconv` (>=0.1.0) plus
  the target framework.
- A trivial `ament_cmake` `CMakeLists.txt` building/installing a placeholder
  library + stub header.

## Distributes to

- **apt** as `ros-<distro>-robotops-trace-bt-cpp` (one deb per ROS distro).

> **Heads-up:** the build depends on `ros-<distro>-robotops-trace-cpp`, which is
> **not yet published**. Per-distro CI will fail at rosdep resolution until the
> SDK core ships. This is expected for the scaffold.
