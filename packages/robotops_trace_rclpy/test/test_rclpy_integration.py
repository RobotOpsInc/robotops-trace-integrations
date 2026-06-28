# Copyright 2026 Robot Ops Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""End-to-end tests for the rclpy monkey-patch integration (ROB-423).

Everything here is REAL: real ``rclpy`` objects (real ``Node``, real
``create_subscription`` / ``create_timer`` / ``create_service`` /
``create_client`` seams, and a real ``example_interfaces/Fibonacci`` action
client->server round trip over the middleware), and the REAL ``robotops`` Python
SDK core driving spans into an OTel ``InMemorySpanExporter`` injected via
``robotops.Config(exporter=...)`` — the same way the SDK core's own tests capture
spans. Assertions are made on the exported spans (name, kind, semconv attributes).

The SDK core ships as ``robotops-trace`` but is not yet on PyPI; install it from
source (the ``development`` branch) into the test environment. When the core is
not importable at all (an environment without it), the span-sink tests skip and
the pure cross-language goal-UUID + idempotency tests still run.
"""

from __future__ import annotations

import time
from contextlib import contextmanager

import pytest

try:
    import robotops
    from opentelemetry.sdk.trace.export.in_memory_span_exporter import InMemorySpanExporter
    from opentelemetry.trace import SpanKind as OTelSpanKind

    _HAS_SDK = True
except ImportError:
    robotops = None  # type: ignore[assignment]
    InMemorySpanExporter = None  # type: ignore[assignment,misc]
    OTelSpanKind = None  # type: ignore[assignment,misc]
    _HAS_SDK = False

import robotops_trace_rclpy
from robotops_trace_rclpy import format_goal_id
from robotops_trace_semconv import (
    ROBOT_ACTION_GOAL_ID,
    ROBOT_ACTION_NAME,
    ROBOT_CALLBACK_TYPE,
    ROBOT_CALLBACK_TYPE_ACTION,
    ROBOT_CALLBACK_TYPE_CLIENT,
    ROBOT_CALLBACK_TYPE_SERVICE,
    ROBOT_CALLBACK_TYPE_SUBSCRIPTION,
    ROBOT_CALLBACK_TYPE_TIMER,
    ROS_SERVICE,
    ROS_TOPIC,
)

# A known goal UUID and its canonical rendering — IDENTICAL literal to the rclcpp
# integration's GoalIdCanonicalFormat gtest (test_rclcpp_integration.cpp). This
# shared constant IS the cross-language contract.
_KNOWN_BYTES = bytes(
    [0xF4, 0x7A, 0xC1, 0x0B, 0x58, 0xCC, 0x43, 0x72,
     0xA5, 0x67, 0x0E, 0x02, 0xB2, 0xC3, 0xD4, 0x79]
)
_KNOWN_CANONICAL = "f47ac10b-58cc-4372-a567-0e02b2c3d479"

# Tests that drive the real SDK span sink. Skipped only when the SDK core is not
# installed; the format + idempotency tests below run regardless.
_needs_sdk = pytest.mark.skipif(
    not _HAS_SDK, reason="robotops SDK core / opentelemetry not installed"
)


# ---------------------------------------------------------------------------
# fixtures
# ---------------------------------------------------------------------------
@pytest.fixture(autouse=True)
def _ensure_installed():
    # Import auto-installs, but be explicit + idempotent.
    robotops_trace_rclpy.install()
    assert robotops_trace_rclpy.is_installed()


@pytest.fixture
def exporter():
    """A real SDK initialised to export into memory (synchronous SimpleSpanProcessor)."""
    exp = InMemorySpanExporter()
    robotops.shutdown()  # clear any prior init (robotops.init is idempotent)
    robotops.init(robotops.Config(service_name="rclpy-test-node", exporter=exp))
    yield exp
    robotops.force_flush()
    robotops.shutdown()


@pytest.fixture(scope="module")
def ros():
    import rclpy

    rclpy.init()
    yield rclpy
    rclpy.shutdown()


@contextmanager
def _node(ros, name):
    node = ros.create_node(name)
    try:
        yield node
    finally:
        node.destroy_node()


def _find(exp, needle):
    return next((s for s in exp.get_finished_spans() if needle in s.name), None)


# ===========================================================================
# Canonical goal-UUID formatting contract (must match the rclcpp side).
# ===========================================================================
def test_goal_id_canonical_format_matches_rclcpp_literal():
    assert format_goal_id(_KNOWN_BYTES) == _KNOWN_CANONICAL


def test_goal_id_matches_manual_rclcpp_algorithm():
    # Replicate rclcpp's goal_id_to_string (identifiers.hpp): lowercase-hex the
    # 16 bytes, then hyphenate 8-4-4-4-12. Proves byte-for-byte parity.
    h = _KNOWN_BYTES.hex()
    manual = f"{h[0:8]}-{h[8:12]}-{h[12:16]}-{h[16:20]}-{h[20:32]}"
    assert format_goal_id(_KNOWN_BYTES) == manual


def test_goal_id_from_uuid_message_and_numpy():
    from unique_identifier_msgs.msg import UUID

    msg = UUID(uuid=list(_KNOWN_BYTES))  # .uuid materialises as a numpy uint8[16]
    assert format_goal_id(msg) == _KNOWN_CANONICAL


# ===========================================================================
# Idempotent patch.
# ===========================================================================
def test_patch_is_idempotent():
    from rclpy.node import Node

    robotops_trace_rclpy.install()
    first = Node.create_subscription
    robotops_trace_rclpy.install()  # second install must not re-wrap
    assert Node.create_subscription is first
    assert getattr(Node.create_subscription, "_robotops_trace_patched", False)


# ===========================================================================
# Per-callback spans (executor instrumentation) — asserted on REAL exported spans.
# ===========================================================================
@_needs_sdk
def test_subscription_callback_opens_span(ros, exporter):
    from std_msgs.msg import String

    ran = {"hit": False}

    def on_msg(msg):
        ran["hit"] = True

    with _node(ros, "sub_test") as node:
        sub = node.create_subscription(String, "/chatter", on_msg, 10)
        # Invoke the (wrapped) callback rclpy stored on the subscription — exactly
        # what the executor calls when a message arrives.
        sub.callback(String())

    robotops.force_flush()
    assert ran["hit"], "user callback must still run"
    span = _find(exporter, "/chatter subscription")
    assert span is not None, f"no span; got {[s.name for s in exporter.get_finished_spans()]}"
    assert span.kind == OTelSpanKind.CONSUMER
    assert span.attributes.get(ROBOT_CALLBACK_TYPE) == ROBOT_CALLBACK_TYPE_SUBSCRIPTION
    assert span.attributes.get(ROS_TOPIC) == "/chatter"


@_needs_sdk
def test_timer_callback_opens_span(ros, exporter):
    ran = {"hit": False}

    with _node(ros, "timer_test") as node:
        timer = node.create_timer(1.0, lambda: ran.__setitem__("hit", True))
        timer.callback()

    robotops.force_flush()
    assert ran["hit"]
    span = _find(exporter, "timer")
    assert span is not None
    assert span.kind == OTelSpanKind.INTERNAL
    assert span.attributes.get(ROBOT_CALLBACK_TYPE) == ROBOT_CALLBACK_TYPE_TIMER


@_needs_sdk
def test_service_callback_opens_span(ros, exporter):
    from example_interfaces.srv import AddTwoInts

    def handle(req, resp):
        resp.sum = req.a + req.b
        return resp

    with _node(ros, "srv_test") as node:
        srv = node.create_service(AddTwoInts, "/add", handle)
        req = AddTwoInts.Request()
        req.a, req.b = 2, 3
        out = srv.callback(req, AddTwoInts.Response())

    robotops.force_flush()
    assert out.sum == 5, "user service callback must run and return its value"
    span = _find(exporter, "/add service")
    assert span is not None
    assert span.kind == OTelSpanKind.SERVER
    assert span.attributes.get(ROBOT_CALLBACK_TYPE) == ROBOT_CALLBACK_TYPE_SERVICE
    assert span.attributes.get(ROS_SERVICE) == "/add"


@_needs_sdk
def test_service_client_call_opens_span(ros, exporter):
    from example_interfaces.srv import AddTwoInts

    with _node(ros, "cli_test") as node:
        client = node.create_client(AddTwoInts, "/add")
        # call_async sends the request (no server needed to observe the span — the
        # CLIENT span opens synchronously around the send).
        client.call_async(AddTwoInts.Request())

    robotops.force_flush()
    span = _find(exporter, "/add call")
    assert span is not None, f"no client span; got {[s.name for s in exporter.get_finished_spans()]}"
    assert span.kind == OTelSpanKind.CLIENT
    assert span.attributes.get(ROBOT_CALLBACK_TYPE) == ROBOT_CALLBACK_TYPE_CLIENT


# ===========================================================================
# Zero robot impact: an instrumentation failure never breaks the callback.
# ===========================================================================
@_needs_sdk
def test_instrumentation_error_does_not_break_callback(ros, monkeypatch):
    from std_msgs.msg import String

    def boom(*a, **k):
        raise RuntimeError("span backend exploded")

    monkeypatch.setattr(robotops, "span", boom)

    ran = {"hit": False}
    with _node(ros, "zero_impact") as node:
        sub = node.create_subscription(String, "/x", lambda m: ran.__setitem__("hit", True), 10)
        sub.callback(String())  # must NOT raise despite robotops.span raising

    assert ran["hit"], "callback must run even when the span backend raises"


# ===========================================================================
# Actions: a REAL client->server round trip emits the SAME canonical goal_id on
# both sides (the deterministic, cross-language ROB-427 join key), captured by the
# real SDK's in-memory exporter.
# ===========================================================================
@_needs_sdk
def test_action_client_server_share_canonical_goal_id(ros, exporter):
    import threading

    from example_interfaces.action import Fibonacci
    from rclpy.action import ActionClient, ActionServer
    from rclpy.executors import SingleThreadedExecutor

    with _node(ros, "action_test") as node:
        def execute_cb(goal_handle):
            result = Fibonacci.Result()
            result.sequence = [0, 1, 1, 2, 3]
            goal_handle.succeed()
            return result

        server = ActionServer(node, Fibonacci, "/fibonacci", execute_cb)
        client = ActionClient(node, Fibonacci, "/fibonacci")

        executor = SingleThreadedExecutor()
        executor.add_node(node)
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()
        try:
            assert client.wait_for_server(timeout_sec=15), "action server not available"

            goal = Fibonacci.Goal()
            goal.order = 5
            send_future = client.send_goal_async(goal)

            deadline = time.time() + 15
            while not send_future.done() and time.time() < deadline:
                time.sleep(0.02)
            goal_handle = send_future.result()
            assert goal_handle is not None and goal_handle.accepted, "goal rejected"

            result_future = goal_handle.get_result_async()
            deadline = time.time() + 15
            while not result_future.done() and time.time() < deadline:
                time.sleep(0.02)
            assert result_future.done(), "action result never arrived"
        finally:
            executor.shutdown()
            spin_thread.join(timeout=5)
            server.destroy()
            client.destroy()

    robotops.force_flush()
    client_span = _find(exporter, "action.goal")
    server_span = _find(exporter, "action.execute")
    names = [s.name for s in exporter.get_finished_spans()]
    assert client_span is not None, f"no client span; got {names}"
    assert server_span is not None, f"no server span; got {names}"

    client_id = client_span.attributes.get(ROBOT_ACTION_GOAL_ID)
    server_id = server_span.attributes.get(ROBOT_ACTION_GOAL_ID)

    # The deterministic cross-process / cross-language join key.
    assert client_id, "client span missing robot.action.goal_id"
    assert server_id, "server span missing robot.action.goal_id"
    assert client_id == server_id, "client and server must emit the SAME goal UUID"
    assert len(client_id) == 36, "expected canonical 8-4-4-4-12 UUID"
    # Re-rendering those bytes the canonical way yields the same string the rclcpp
    # integration would emit -> the two languages stitch.
    assert format_goal_id(bytes.fromhex(client_id.replace("-", ""))) == client_id

    # Other semconv attributes + the direction signal (matches rclcpp ROB-427).
    assert client_span.attributes.get(ROBOT_ACTION_NAME) == "/fibonacci"
    assert server_span.attributes.get(ROBOT_ACTION_NAME) == "/fibonacci"
    assert server_span.attributes.get(ROBOT_CALLBACK_TYPE) == ROBOT_CALLBACK_TYPE_ACTION
    assert client_span.kind == OTelSpanKind.CLIENT
    assert server_span.kind == OTelSpanKind.SERVER
