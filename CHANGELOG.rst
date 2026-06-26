^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for robotops-trace-integrations
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This monorepo contains independently-versioned packages. Each package owns its
``package.xml`` version; ``version-check.yml`` runs per changed package. Entries
below are tagged with the affected package.

0.1.0 (2026-06-26)
-------------------

* Scaffold the robotops-trace-integrations monorepo (colcon workspace).
* (robotops_trace_semconv) ROB-430 stub: header-only C++ + Python mirror of the
  robotics-general semantic-convention keys.
* (robotops_trace_rclcpp) ROB-422 stub: ament_cmake placeholder package.
* (robotops_trace_rclpy) ROB-423 stub: ament_python placeholder package (+ PyPI lane).
* (robotops_trace_bt_cpp) ROB-424 stub: ament_cmake placeholder package.
* (robotops_trace_ros2_control) ROB-425 stub: ament_cmake placeholder package.
* (robotops_trace_moveit) ROB-426 stub: ament_cmake placeholder package (★ async patch carrier).
* CI/CD scaffold: per-distro Docker matrix CI, branch-name validation, per-package
  version-check, bloom→deb release / release-dev, PyPI publish lane, shared
  publish-debian-s3 action.
