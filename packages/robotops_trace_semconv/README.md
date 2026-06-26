# robotops_trace_semconv

**Issue:** [ROB-430](https://linear.app/robotops/issue/ROB-430) &nbsp;•&nbsp; **Status:** 🟡 stub / scaffold

Semantic conventions for RobotOps Trace: the robotics-general span/attribute keys
shared by every integration package and the SDK cores. Header-only C++
(`include/robotops_trace_semconv/semconv.hpp`) plus a mirrored Python module
(`robotops_trace_semconv/`). This is the base of the integrations dependency
graph and the lockstep contract that keeps attribute names consistent.

This package ships **two ways**:

- **apt** (`ros-<distro>-robotops-trace-semconv`) via `ament_cmake` for ROS fleets.
- **PyPI** (`robotops-trace-semconv`) — the Python mirror, for pip-only users.

## What's here now (scaffold)

- Header-only `INTERFACE` CMake target installing the stub header.
- Python mirror module installed via `ament_python_install_package`.
- Concept-level placeholder keys only: `robot.action.result`,
  `robot.transform.parent`/`child`, `robot.joint.name`,
  `robot.trajectory.point_count` — all marked `TODO(ROB-430)`.

## What ROB-430 fills in

The authoritative key registry, value types/units, the namespacing scheme, and
the ROS-to-semconv mapping table (rclcpp_action goal UUID, FollowJointTrajectory
boundary, BT node identity, etc.). The C++ header and Python module MUST stay
byte-for-byte aligned.
