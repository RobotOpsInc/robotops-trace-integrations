# robotops_trace_ros2_control

**Issue:** [ROB-425](https://linear.app/robotops/issue/ROB-425) &nbsp;•&nbsp; **Status:** 🟡 stub / scaffold

RobotOps Trace integration for **ros2_control**.

Instruments the controller / FollowJointTrajectory boundary in an RT-safe way. Built on the `robotops_trace_cpp` SDK core. **ROB-425** fills in the real controller hooks.

## What's here now (scaffold)

- `package.xml` (Format 3, v0.1.0, Apache-2.0) declaring the dependency on
  `robotops_trace_cpp` (>=0.1.0) and `robotops_trace_semconv` (>=0.1.0) plus
  the target framework.
- A trivial `ament_cmake` `CMakeLists.txt` building/installing a placeholder
  library + stub header.

## Distributes to

- **apt** as `ros-<distro>-robotops-trace-ros2-control` (one deb per ROS distro).

> **Heads-up:** the build depends on `ros-<distro>-robotops-trace-cpp`, which is
> **not yet published**. Per-distro CI will fail at rosdep resolution until the
> SDK core ships. This is expected for the scaffold.
