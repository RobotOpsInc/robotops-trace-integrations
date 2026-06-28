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

"""Zero-impact span helpers shared by every rclpy wrapper.

DESIGN — these helpers are the single choke point through which all callback /
action wrappers open spans, so the *zero-robot-impact* guarantee lives in one
place:

  * Spans are opened through the public ``robotops`` Python SDK (``robotops.span``
    — the same surface the SDK doc-strings advertise). The SDK owns whether a span
    is real or a no-op; if the SDK is uninitialised / disabled, ``robotops.span``
    is already a transparent no-op, so the wrappers add nothing.
  * An *instrumentation* error (building attributes, the SDK raising, even
    ``__enter__`` / ``__exit__`` raising) is swallowed — it must NEVER propagate
    into the user's callback. A user-callback error, by contrast, is propagated
    untouched (we never suppress it).
  * Span *kind* is passed as the ``kind=`` keyword and is forward-compatible: if a
    given SDK build doesn't accept ``kind``, we transparently retry without it.

Intra-process nesting (a callback span nesting under whatever span is already
active on the thread) is delegated to the SDK's contextvar-based current-span
tracking — opening via ``robotops.span`` is all that is required.
"""

from __future__ import annotations

from typing import Any

try:
    import robotops  # the RobotOps Python tracing SDK core
except ImportError:
    # The SDK core isn't installed (e.g. CI before robotops-trace publishes). The
    # integration then degrades to a transparent pass-through — patching still
    # happens and is idempotent, the wrappers just open no spans. This IS part of
    # the zero-impact contract: no SDK -> no tracing, never an error.
    robotops = None  # type: ignore[assignment]

__all__ = [
    "SPAN_KIND_INTERNAL",
    "SPAN_KIND_SERVER",
    "SPAN_KIND_CLIENT",
    "SPAN_KIND_CONSUMER",
    "safe_span",
]

# Language-neutral span-kind tokens, mirroring the rclcpp integration's use of
# robotops::SpanKind (Server on the action-server side, Client on the
# action-client side — the ROB-427 direction signal). Passed to the SDK via
# ``kind=``.
SPAN_KIND_INTERNAL = "internal"
SPAN_KIND_SERVER = "server"
SPAN_KIND_CLIENT = "client"
SPAN_KIND_CONSUMER = "consumer"


def _open(name: str, kind: str | None, attributes: dict[str, Any]):
    """Open a span via the SDK, tolerating SDK builds without a ``kind=`` kwarg."""
    if robotops is None:
        return None  # SDK core absent -> transparent pass-through
    if kind is not None:
        try:
            return robotops.span(name, kind=kind, **attributes)
        except TypeError:
            # This SDK build doesn't accept kind= — fall back to attrs only.
            pass
    return robotops.span(name, **attributes)


class _SafeSpan:
    """A context manager that opens a span but can never break the user's code.

    All instrumentation failures degrade to a transparent pass-through. User
    exceptions raised inside the ``with`` body are propagated (never suppressed).
    """

    __slots__ = ("_cm",)

    def __init__(self, name: str, kind: str | None, attributes: dict[str, Any]) -> None:
        self._cm: Any = None
        try:
            self._cm = _open(name, kind, attributes)
        except Exception:
            self._cm = None

    def __enter__(self) -> None:
        if self._cm is not None:
            try:
                self._cm.__enter__()
            except Exception:
                self._cm = None
        return None

    def __exit__(self, exc_type: Any, exc: Any, tb: Any) -> bool:
        if self._cm is not None:
            try:
                self._cm.__exit__(exc_type, exc, tb)
            except Exception:
                pass
        return False  # never suppress a user-callback exception


def safe_span(name: str, kind: str | None = None, **attributes: Any) -> _SafeSpan:
    """Open a zero-impact span. Always returns a usable context manager."""
    return _SafeSpan(name, kind, attributes)
