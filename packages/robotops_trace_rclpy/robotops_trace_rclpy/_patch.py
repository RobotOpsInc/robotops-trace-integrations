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

"""Monkey-patch rclpy so stock Python ROS 2 nodes get traced — no fork.

The customer does ``import robotops_trace_rclpy`` (which auto-installs) and their
existing rclpy nodes start emitting spans. We patch the equivalent seams to the
C++ ``rclcpp`` integration, at the public ``rclpy`` API surface:

  * ``rclpy.node.Node.create_subscription`` — wrap the user callback in a span
    (``robot.callback.type=subscription``).
  * ``rclpy.node.Node.create_timer`` — wrap the timer callback
    (``robot.callback.type=timer``).
  * ``rclpy.node.Node.create_service`` — wrap the service callback, SERVER kind
    (``robot.callback.type=service``).
  * ``rclpy.node.Node.create_client`` — wrap the returned client's
    ``call_async`` / ``call``, CLIENT kind (``robot.callback.type=client``).
  * ``rclpy.action.ActionClient.send_goal_async`` — CLIENT span carrying
    ``robot.action.goal_id`` (canonical UUID; covers the sync ``send_goal`` too,
    which delegates to it).
  * ``rclpy.action.ActionServer.__init__`` — wrap the ``execute_callback`` in a
    SERVER span carrying ``robot.action.goal_id`` — the SAME canonical UUID the
    client emits, so the ROB-427 join stitches Python<->C++ in either direction.

Guarantees:
  * **Idempotent** — installing twice (or importing twice) does not double-wrap;
    each patched attribute carries a sentinel.
  * **Zero robot impact** — every patch and wrapper is wrapped in defensive
    error handling; an instrumentation failure can never break the user's node.
    Span opening itself is delegated to the always-safe helpers in ``_spans``.
"""

from __future__ import annotations

import asyncio
import functools
from typing import Any

from robotops_trace_semconv import (
    ROBOT_ACTION_GOAL_ID,
    ROBOT_ACTION_NAME,
    ROBOT_CALLBACK_TYPE,
    ROBOT_CALLBACK_TYPE_CLIENT,
    ROBOT_CALLBACK_TYPE_SERVICE,
    ROBOT_CALLBACK_TYPE_SUBSCRIPTION,
    ROBOT_CALLBACK_TYPE_TIMER,
    ROS_MESSAGE_TYPE,
    ROS_SERVICE,
    ROS_TOPIC,
)

from ._identifiers import format_goal_id
from ._spans import (
    SPAN_KIND_CLIENT,
    SPAN_KIND_CONSUMER,
    SPAN_KIND_INTERNAL,
    SPAN_KIND_SERVER,
    safe_span,
)

__all__ = ["install", "uninstall", "is_installed"]

# Sentinel marking a callable we have already wrapped/patched, so a second
# install() (or a second wrap of the same callback) is a no-op.
_PATCHED = "_robotops_trace_patched"

# Saved originals, so uninstall() can fully restore stock rclpy.
_originals: dict[str, Any] = {}
_installed = False


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------
def _msg_type_name(msg_type: Any) -> str:
    """Best-effort ROS-style type name, e.g. ``std_msgs/msg/String``."""
    try:
        module = getattr(msg_type, "__module__", "") or ""
        name = getattr(msg_type, "__name__", "") or ""
        # 'std_msgs.msg._string' -> 'std_msgs/msg' (drop the private leaf module)
        parts = [p for p in module.split(".") if not p.startswith("_")]
        return "/".join([*parts, name]) if name else module
    except Exception:
        return ""


def _wrap_callback(
    callback: Any,
    span_name: str,
    kind: str | None,
    attributes: dict[str, Any],
) -> Any:
    """Wrap a callback so each invocation opens a span; idempotent + zero-impact.

    Supports both plain functions and coroutine functions (rclpy executes either):
    for a coroutine callback the span is held across the ``await`` so it spans the
    whole logical unit of work.
    """
    if callback is None or getattr(callback, _PATCHED, False):
        return callback

    if asyncio.iscoroutinefunction(callback):

        @functools.wraps(callback)
        async def async_wrapper(*args: Any, **kwargs: Any) -> Any:
            with safe_span(span_name, kind, **attributes):
                return await callback(*args, **kwargs)

        wrapper: Any = async_wrapper
    else:

        @functools.wraps(callback)
        def sync_wrapper(*args: Any, **kwargs: Any) -> Any:
            with safe_span(span_name, kind, **attributes):
                return callback(*args, **kwargs)

        wrapper = sync_wrapper

    try:
        setattr(wrapper, _PATCHED, True)
    except Exception:
        pass
    return wrapper


# ---------------------------------------------------------------------------
# Node callback seams
# ---------------------------------------------------------------------------
def _patch_node() -> None:
    from rclpy.node import Node

    orig_create_subscription = Node.create_subscription
    orig_create_timer = Node.create_timer
    orig_create_service = Node.create_service
    orig_create_client = Node.create_client

    if getattr(orig_create_subscription, _PATCHED, False):
        return  # already patched

    @functools.wraps(orig_create_subscription)
    def create_subscription(self, msg_type, topic, callback, *args, **kwargs):  # type: ignore[no-untyped-def]
        try:
            callback = _wrap_callback(
                callback,
                f"{topic} subscription",
                SPAN_KIND_CONSUMER,
                {
                    ROBOT_CALLBACK_TYPE: ROBOT_CALLBACK_TYPE_SUBSCRIPTION,
                    ROS_TOPIC: topic,
                    ROS_MESSAGE_TYPE: _msg_type_name(msg_type),
                },
            )
        except Exception:
            pass  # never block subscription creation
        return orig_create_subscription(self, msg_type, topic, callback, *args, **kwargs)

    @functools.wraps(orig_create_timer)
    def create_timer(self, timer_period_sec, callback, *args, **kwargs):  # type: ignore[no-untyped-def]
        try:
            callback = _wrap_callback(
                callback,
                "timer",
                SPAN_KIND_INTERNAL,
                {ROBOT_CALLBACK_TYPE: ROBOT_CALLBACK_TYPE_TIMER},
            )
        except Exception:
            pass
        return orig_create_timer(self, timer_period_sec, callback, *args, **kwargs)

    @functools.wraps(orig_create_service)
    def create_service(self, srv_type, srv_name, callback, *args, **kwargs):  # type: ignore[no-untyped-def]
        try:
            callback = _wrap_callback(
                callback,
                f"{srv_name} service",
                SPAN_KIND_SERVER,
                {
                    ROBOT_CALLBACK_TYPE: ROBOT_CALLBACK_TYPE_SERVICE,
                    ROS_SERVICE: srv_name,
                },
            )
        except Exception:
            pass
        return orig_create_service(self, srv_type, srv_name, callback, *args, **kwargs)

    @functools.wraps(orig_create_client)
    def create_client(self, srv_type, srv_name, *args, **kwargs):  # type: ignore[no-untyped-def]
        client = orig_create_client(self, srv_type, srv_name, *args, **kwargs)
        try:
            _wrap_client_call(client, srv_name)
        except Exception:
            pass  # never break client creation
        return client

    for fn in (create_subscription, create_timer, create_service, create_client):
        setattr(fn, _PATCHED, True)

    Node.create_subscription = create_subscription  # type: ignore[method-assign]
    Node.create_timer = create_timer  # type: ignore[method-assign]
    Node.create_service = create_service  # type: ignore[method-assign]
    Node.create_client = create_client  # type: ignore[method-assign]

    _originals["Node.create_subscription"] = orig_create_subscription
    _originals["Node.create_timer"] = orig_create_timer
    _originals["Node.create_service"] = orig_create_service
    _originals["Node.create_client"] = orig_create_client


def _wrap_client_call(client: Any, srv_name: str) -> None:
    """Wrap a service client's ``call_async`` / ``call`` to emit a CLIENT span."""
    for method_name in ("call_async", "call"):
        method = getattr(client, method_name, None)
        if method is None or getattr(method, _PATCHED, False):
            continue
        attrs = {
            ROBOT_CALLBACK_TYPE: ROBOT_CALLBACK_TYPE_CLIENT,
            ROS_SERVICE: srv_name,
        }
        wrapped = _wrap_callback(method, f"{srv_name} call", SPAN_KIND_CLIENT, attrs)
        try:
            setattr(client, method_name, wrapped)
        except Exception:
            pass


# ---------------------------------------------------------------------------
# Action seams — the cross-language goal-UUID key
# ---------------------------------------------------------------------------
def _new_goal_uuid() -> Any:
    """A fresh ``unique_identifier_msgs/UUID`` (random), matching rclpy's own."""
    import uuid as _uuid

    from unique_identifier_msgs.msg import UUID

    return UUID(uuid=list(_uuid.uuid4().bytes))


def _patch_action_client() -> None:
    from rclpy.action import ActionClient

    orig_send_goal_async = ActionClient.send_goal_async
    if getattr(orig_send_goal_async, _PATCHED, False):
        return

    @functools.wraps(orig_send_goal_async)
    def send_goal_async(self, goal, feedback_callback=None, goal_uuid=None):  # type: ignore[no-untyped-def]
        # Materialise the goal UUID up-front (rclpy would otherwise generate one
        # internally) so the CLIENT span carries the SAME id the SERVER will see.
        attrs: dict[str, Any] = {ROBOT_CALLBACK_TYPE: ROBOT_CALLBACK_TYPE_CLIENT}
        try:
            if goal_uuid is None:
                goal_uuid = _new_goal_uuid()
            attrs[ROBOT_ACTION_GOAL_ID] = format_goal_id(goal_uuid)
            attrs[ROBOT_ACTION_NAME] = getattr(self, "_action_name", "")
        except Exception:
            goal_uuid = None  # fall back to rclpy's own generation; no goal_id key
        name = f"{getattr(self, '_action_name', 'action')} action.goal"
        with safe_span(name, SPAN_KIND_CLIENT, **attrs):
            return orig_send_goal_async(
                self, goal, feedback_callback=feedback_callback, goal_uuid=goal_uuid
            )

    setattr(send_goal_async, _PATCHED, True)
    ActionClient.send_goal_async = send_goal_async  # type: ignore[method-assign]
    _originals["ActionClient.send_goal_async"] = orig_send_goal_async


def _patch_action_server() -> None:
    from rclpy.action import ActionServer

    orig_init = ActionServer.__init__
    if getattr(orig_init, _PATCHED, False):
        return

    @functools.wraps(orig_init)
    def __init__(self, node, action_type, action_name, execute_callback=None, **kwargs):  # type: ignore[no-untyped-def]
        try:
            if execute_callback is not None:
                execute_callback = _wrap_execute_callback(execute_callback, action_name)
        except Exception:
            pass  # never block server construction
        orig_init(self, node, action_type, action_name, execute_callback, **kwargs)

    setattr(__init__, _PATCHED, True)
    ActionServer.__init__ = __init__  # type: ignore[method-assign]
    _originals["ActionServer.__init__"] = orig_init


def _wrap_execute_callback(execute_callback: Any, action_name: str) -> Any:
    """Wrap an action ``execute_callback`` in a SERVER span carrying the goal UUID.

    The ``goal_handle`` passed to the execute callback exposes ``.goal_id`` (the
    same 16-byte UUID the client sent), which we render canonically — this is the
    cross-process / cross-language join key with the rclcpp server.
    """
    if execute_callback is None or getattr(execute_callback, _PATCHED, False):
        return execute_callback

    def _attrs(goal_handle: Any) -> dict[str, Any]:
        attrs: dict[str, Any] = {
            ROBOT_CALLBACK_TYPE: "action",
            ROBOT_ACTION_NAME: action_name,
        }
        try:
            attrs[ROBOT_ACTION_GOAL_ID] = format_goal_id(goal_handle.goal_id)
        except Exception:
            pass
        return attrs

    name = f"{action_name} action.execute"

    if asyncio.iscoroutinefunction(execute_callback):

        @functools.wraps(execute_callback)
        async def async_wrapper(goal_handle: Any) -> Any:
            with safe_span(name, SPAN_KIND_SERVER, **_attrs(goal_handle)):
                return await execute_callback(goal_handle)

        wrapper: Any = async_wrapper
    else:

        @functools.wraps(execute_callback)
        def sync_wrapper(goal_handle: Any) -> Any:
            with safe_span(name, SPAN_KIND_SERVER, **_attrs(goal_handle)):
                return execute_callback(goal_handle)

        wrapper = sync_wrapper

    try:
        setattr(wrapper, _PATCHED, True)
    except Exception:
        pass
    return wrapper


# ---------------------------------------------------------------------------
# public install / uninstall
# ---------------------------------------------------------------------------
def install() -> None:
    """Install the rclpy trace patches. Idempotent and best-effort.

    Patches each public seam independently; a failure to patch one seam (e.g. an
    rclpy version without it) never prevents the others or raises to the caller.
    """
    global _installed
    if _installed:
        return
    for patcher in (_patch_node, _patch_action_client, _patch_action_server):
        try:
            patcher()
        except Exception:
            # Best-effort: a missing/renamed seam must not break import or the
            # rest of the patch set.
            pass
    _installed = True


def uninstall() -> None:
    """Restore stock rclpy (used by tests). Best-effort."""
    global _installed
    try:
        from rclpy.node import Node

        if "Node.create_subscription" in _originals:
            Node.create_subscription = _originals["Node.create_subscription"]  # type: ignore[method-assign]
        if "Node.create_timer" in _originals:
            Node.create_timer = _originals["Node.create_timer"]  # type: ignore[method-assign]
        if "Node.create_service" in _originals:
            Node.create_service = _originals["Node.create_service"]  # type: ignore[method-assign]
        if "Node.create_client" in _originals:
            Node.create_client = _originals["Node.create_client"]  # type: ignore[method-assign]
    except Exception:
        pass
    try:
        from rclpy.action import ActionClient, ActionServer

        if "ActionClient.send_goal_async" in _originals:
            ActionClient.send_goal_async = _originals["ActionClient.send_goal_async"]  # type: ignore[method-assign]
        if "ActionServer.__init__" in _originals:
            ActionServer.__init__ = _originals["ActionServer.__init__"]  # type: ignore[method-assign]
    except Exception:
        pass
    _originals.clear()
    _installed = False


def is_installed() -> bool:
    """Whether the patches are currently installed."""
    return _installed
