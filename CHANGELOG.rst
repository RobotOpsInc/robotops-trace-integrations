^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for robotops-trace-integrations
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This monorepo contains independently-versioned packages. Each package owns its
``package.xml`` version; ``version-check.yml`` runs per changed package. Entries
below are tagged with the affected package.

0.2.0 (2026-06-27)
------------------

* (robotops_trace_rclpy) ROB-423: first real rclpy integration — Python node
  parity with the rclcpp integration, promoting the package from a stub. It
  **monkey-patches stock rclpy at import** (``import robotops_trace_rclpy``), so
  existing Python ROS 2 nodes get traced with no code changes and no fork. Built
  on the ``robotops-trace`` Python SDK core and the shared
  ``robotops_trace_semconv`` keys.

  * **Action goal-UUID correlation (the deterministic, cross-language win).**
    ``ActionClient.send_goal_async`` (and the sync ``send_goal`` that delegates to
    it) opens a CLIENT span and ``ActionServer``'s ``execute_callback`` a SERVER
    span, both carrying ``robot.action.goal_id`` — the canonical RFC-4122
    8-4-4-4-12 lowercase UUID, rendered **byte-identically** to the rclcpp
    integration's ``goal_id_to_string`` (via ``uuid.UUID(bytes=...)``). So the
    ROB-427 agent join stitches a Python client to a C++ server (or vice versa).
  * **Per-callback spans (executor instrumentation).** Patching
    ``Node.create_subscription`` / ``create_timer`` / ``create_service`` /
    ``create_client`` wraps the user callback at creation time (the clean public
    hook — rclpy's executor dispatch is not a public seam). Each invocation opens
    a span tagged with ``robot.callback.type``
    (subscription/timer/service/client). Automatic on import for every node in
    the process; nesting delegated to the SDK's contextvar current-span tracking.
  * **Zero robot impact + idempotent patch.** Wrappers never raise into user
    code (instrumentation errors are swallowed; user-callback errors propagate
    untouched); transparent pass-through when the SDK is uninitialised; importing
    twice never double-wraps (sentinel-guarded); ``uninstall()`` restores stock
    rclpy. Span kind is passed to the SDK as a real ``robotops.SpanKind``
    (CLIENT on the action client, SERVER on the action server, CONSUMER on
    subscriptions, …).
  * **Limitation:** rclpy does not surface ``rmw_message_info`` to subscription
    callbacks, so the ``ros.publisher_gid`` / ``ros.source_timestamp`` content
    keys the rclcpp integration emits are not available fork-free in rclpy today;
    ``ros.topic`` / ``ros.message.type`` are emitted and the gap is documented.
  * Verified end-to-end in ``ros:jazzy`` Docker against the REAL ``robotops``
    Python SDK core (installed from the ``robotops-trace-python`` ``development``
    branch) + semconv: the suite injects an OTel ``InMemorySpanExporter`` via
    ``robotops.Config`` and asserts on the spans the integration actually produces
    (names, kind, semconv attributes), including a real
    ``example_interfaces/Fibonacci`` action round trip proving both sides emit the
    same canonical goal UUID — 10 passed. The Python core has no apt/PyPI release
    yet, so the CI image builds without it (skip-keyed): the SDK-sink tests skip in
    CI while the pure goal-UUID + idempotency tests run.

* (robotops_trace_ros2_control) ROB-425: first real ros2_control integration —
  RT-safe instrumentation of the controller / ``FollowJointTrajectory``
  action-server boundary, promoting the package from a stub. Built on the
  ``robotops_trace_cpp`` SDK core (>=0.3.0, for the new detached-span API) and the
  shared ``robotops_trace_semconv`` keys.

  * **One detached SERVER span per goal (accept → result).** A new helper,
    ``robotops::trace::ros2_control::FollowJointTrajectoryTracer``, opens a span
    on goal-accept and closes it on the terminal result, carrying
    ``robot.action.goal_id`` (the canonical RFC-4122 8-4-4-4-12 lowercase UUID,
    rendered **byte-identically** to the rclcpp/rclpy action client's
    ``goal_id_to_string``, so the ROB-427 agent join stitches the controller hop
    under the action client), ``robot.action.name`` / ``robot.action.result``,
    and the joint/trajectory semconv (``robot.joint.name`` comma-joined +
    ``robot.joint.count`` + ``robot.trajectory.point_count``) read from the goal.
  * **Detached span (ROB-443) — the right primitive for the async hold.**
    ``robotops::start_detached_span()`` mints the span **without** touching any
    thread's current-context, so it survives from accept to result across the
    controller's async execution (RT loop + non-RT monitor) and is ended on
    whatever thread observes the terminal, with an explicit parent.
  * **RT-safety (hard guarantee).** Nothing runs in the real-time ``update()``
    control loop — no span op, no allocation, no lock, no log. All hooks run on
    the executor / non-RT monitor thread; the terminal outcome is decided in
    ``update()`` the stock ros2_control way (an RT-safe ``RealtimeServerGoalHandle``
    flag write) and the span is closed later, non-RT.
  * **No-fork delivery: carry-patch.** Stock ``joint_trajectory_controller``
    creates its action server internally, so the package ships both the reusable
    helper **and** a carried patch (``patches/``) wiring it into stock JTC at the
    accept/result boundary — the same carry-patch-now → upstream-later model as
    the MoveIt async patch (spec §3.4). Customers use our patched JTC or call the
    helper directly from a custom controller.
  * **Zero-robot-impact.** Every helper method is ``noexcept`` + catch-all
    wrapped; detached-span ops are noexcept and no-op when the SDK is disabled.
  * **Tested:** a focused gtest drives a real ``control_msgs`` FollowJointTrajectory
    goal through the helper's accept→result path and asserts via the core
    ``InMemorySpanExporter`` that exactly one SERVER span carries the canonical
    ``robot.action.goal_id`` + the joint/trajectory semconv + the correct result
    status, nested under an explicit parent — 5 passed, ament lints green. The
    carried patch is proven to **apply cleanly** (``git apply --check``) and
    **compile** against stock ros2_controllers 4.40.1 (jazzy). The full
    ``controller_manager`` + hardware end-to-end is deferred to on-hardware
    ROB-435.

* (robotops_trace_bt_cpp) ROB-424: first real BehaviorTree.CPP integration —
  opt-in, fork-free instrumentation that covers both Nav2 and MoveIt Pro (both
  run upstream BT.CPP), built on the ``robotops_trace_cpp`` SDK core.

  * **Mechanism: the public ``BT::StatusChangeLogger`` seam.** ``TreeTracer`` is
    a small ``StatusChangeLogger`` subclass; the customer constructs one after
    building their tree (``robotops::trace::bt::TreeTracer tracer(tree);``). Its
    base ctor subscribes to every node's status-change signal — **no fork of
    BehaviorTree.CPP, no changes to user nodes**.
  * **One span per node EXECUTION, not per tick.** BT.CPP re-ticks at 10–100 Hz;
    a span is opened when a node enters execution (``IDLE -> RUNNING``, or
    ``IDLE -> terminal`` for a synchronous node) and closed on the terminal
    status (``SUCCESS`` / ``FAILURE`` / halt-to-``IDLE`` / ``SKIPPED``), so a
    long-running async node yields one span covering its whole execution, not a
    per-tick flood.
  * **Nesting by TREE STRUCTURE, not thread-local context.** At attach time the
    tree is walked once to build a child-UID -> parent-UID map; each span opens
    with an explicit parent (``robotops::SpanOptions.parent``) set to the parent
    node's still-open span. Correct because a control node goes ``RUNNING``
    before ticking its children. This is robust to async nodes ticked across many
    call stacks/threads, where thread-local context would be wrong.
  * **Attributes.** Span name = ``node.name()``; ``robot.component.name``
    (semconv) = node name; ``bt.node_type`` = ``registrationName()`` and
    ``bt.status`` = terminal status are BT-local (not in semconv v0; a
    ``TODO(semconv-v1)`` flags ``robot.behavior.*`` as the promotion candidate —
    semconv is not edited here). ``FAILURE`` maps to span ``StatusCode::Error``,
    ``SUCCESS`` to ``Ok``.
  * **Zero-robot-impact.** The status-change callback never throws/blocks (SDK
    span ops are ``noexcept`` + a catch-all); a disabled/uninitialised SDK is a
    cheap no-op that does not perturb the tick.
  * Verified by a gtest suite in a combined ``ros:jazzy`` colcon workspace (core
    from ``apt.development``, semconv from source, ``behaviortree_cpp`` from apt):
    a ``Sequence`` of an async ``StatefulActionNode`` + a sync action, ticked to
    completion, proves one span per executed node, both leaves nested under the
    Sequence (incl. the async node across ticks), a ``FAILURE`` node yielding an
    Error-status span, and a kill-switched SDK emitting no spans. Spans captured
    via the core's ``InMemorySpanExporter``; ament lints green.

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
