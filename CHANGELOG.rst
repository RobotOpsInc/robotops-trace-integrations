^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for robotops-trace-integrations
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This monorepo contains independently-versioned packages. Each package owns its
``package.xml`` version; ``version-check.yml`` runs per changed package. Entries
below are tagged with the affected package.

0.2.0 (2026-06-26)
------------------

* (robotops_trace_semconv) ROB-430: robotics semantic conventions v0 — the real
  dictionary, promoting the package from a stub to the authoritative source of
  truth. Header-only C++ (``robotops::trace::semconv``) + a byte-for-byte Python
  mirror define every v0 key in two namespaces: ``robot.*`` portable concept keys
  (action, callback, transform, joint, trajectory, target/pose, object,
  component) and ``ros.*`` ROS-mapping keys (node, topic, service, message type,
  publisher gid, source timestamp, content hash), plus the ``service.name`` /
  ``robot.id`` resource attributes and the enumerated value constants
  (``robot.action.status`` / ``robot.action.result`` / ``robot.callback.type``).
  README carries the full dictionary table; a pytest smoke test asserts the key
  strings. v0 is additive-only.

* (robotops_trace_rclcpp) ROB-430: source span-attribute keys from
  ``robotops_trace_semconv`` instead of local string literals. The rclcpp
  integration now re-exports ``robot.action.goal_id``, ``robot.action.name``,
  ``robot.action.result``, ``ros.topic``, ``ros.publisher_gid`` and
  ``ros.source_timestamp`` from the dictionary — the emitted strings are
  unchanged, so the ROB-422/427 goal-UUID correlation contract is preserved
  (verified by the existing gtest suite). ``robot.action.goal_response`` /
  ``robot.action.goal_accepted`` remain rclcpp-local (not in semconv v0).

* (robotops_trace_rclcpp) ROB-422: first real rclcpp integration — opt-in,
  fork-free instrumentation for C++ ROS 2 nodes, built on the
  ``robotops_trace_cpp`` SDK core.

  * **rclcpp_action goal-UUID correlation (the deterministic win).**
    ``create_traced_action_server`` and ``trace_send_goal_options`` /
    ``send_traced_goal`` wrap the PUBLIC ``rclcpp_action`` server callbacks and
    client ``SendGoalOptions``. Every goal produces server- and client-side
    spans carrying ``robot.action.goal_id`` — the canonical 8-4-4-4-12 lowercase
    UUID (``goal_id_to_string``), emitted identically on both sides so the
    correlation agent (ROB-427) can join the two traces across the process
    boundary. ``scoped_action_span`` opens a Server span over the full goal
    execution from inside the user's ``execute()`` body.
  * **Per-callback spans (executor instrumentation).** ``traced_callback`` and
    ``traced_subscription`` wrap subscription/timer/service callbacks at creation
    time (the clean public hook in jazzy — no rclcpp fork). Each invocation opens
    a span nested under the active thread-local context. OPT-IN per callback;
    process-wide auto-instrumentation is deferred to the ROB-421 auto-init layer.
  * **Best-effort content-correlation keys.** ``record_message_info`` /
    ``traced_subscription`` stamp the publisher GID and source timestamp from
    ``rclcpp::MessageInfo`` (``ros.publisher_gid``, ``ros.source_timestamp``,
    ``ros.topic``) onto subscription spans.
  * Local concept-level attribute keys (``robot.action.*``, ``ros.*``) pending
    migration to ``robotops_trace_semconv`` (ROB-430); the ``robot.action.result``
    key is already re-exported from semconv.
  * Verified by a gtest suite in a combined ``ros:jazzy`` colcon workspace
    (core-from-source): a real action client→server round trip proves both sides
    emit the same goal UUID, and a wrapped subscription callback nests under the
    active context and carries the content keys. Spans captured via the core's
    ``InMemorySpanExporter``.
  * NOTE: the SDK core was built from source. The integration CI (which resolves
    ``ros-<distro>-robotops-trace-cpp`` from apt) stays red until the core
    publishes to ``apt.development``; the local colcon build is the verification.

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
