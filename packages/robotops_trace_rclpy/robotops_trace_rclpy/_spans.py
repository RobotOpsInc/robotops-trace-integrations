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

  * Spans are opened through the public ``robotops`` Python SDK core, using its
    real signature: ``robotops.span(name, *, kind=SpanKind, attributes=dict)``,
    which returns a context manager (usable as ``with`` and ``async with``)
    yielding a ``Span`` handle. The SDK itself is no-op-safe when
    uninitialised/disabled, so when tracing is off the wrappers add nothing.
  * If the SDK core is *not importable at all* (e.g. an environment without it
    installed), the helpers degrade to a transparent pass-through — patching
    still happens and stays idempotent, the wrappers just open no spans.
  * An *instrumentation* error (building attributes, the SDK raising, even
    ``__enter__`` / ``__exit__`` raising) is swallowed — it must NEVER propagate
    into the user's callback. A user-callback error, by contrast, is propagated
    untouched (we never suppress it).

Intra-process nesting (a callback span nesting under whatever span is already
active on the thread/task) is delegated to the SDK's contextvar-based current
context — opening via ``robotops.span`` is all that is required, and it carries
across ``await`` within a task.
"""

from __future__ import annotations

from typing import Any

try:
    import robotops  # the RobotOps Python tracing SDK core
    from robotops import SpanKind as _SpanKind

    SPAN_KIND_INTERNAL: Any = _SpanKind.INTERNAL
    SPAN_KIND_SERVER: Any = _SpanKind.SERVER
    SPAN_KIND_CLIENT: Any = _SpanKind.CLIENT
    SPAN_KIND_CONSUMER: Any = _SpanKind.CONSUMER
except ImportError:
    # The SDK core isn't installed. The integration then degrades to a
    # transparent pass-through; this IS part of the zero-impact contract: no SDK
    # -> no tracing, never an error.
    robotops = None  # type: ignore[assignment]
    SPAN_KIND_INTERNAL = SPAN_KIND_SERVER = SPAN_KIND_CLIENT = SPAN_KIND_CONSUMER = None

__all__ = [
    "SPAN_KIND_INTERNAL",
    "SPAN_KIND_SERVER",
    "SPAN_KIND_CLIENT",
    "SPAN_KIND_CONSUMER",
    "safe_span",
]


def _open(name: str, kind: Any, attributes: dict[str, Any]):
    """Open a span via the SDK's real API, or return None when the SDK is absent."""
    if robotops is None:
        return None  # SDK core not importable -> transparent pass-through
    if kind is None:
        return robotops.span(name, attributes=attributes)
    return robotops.span(name, kind=kind, attributes=attributes)


class _SafeSpan:
    """A context manager (sync + async) that opens a span but can never break the
    user's code.

    All instrumentation failures degrade to a transparent pass-through. User
    exceptions raised inside the body are propagated (never suppressed).
    """

    __slots__ = ("_cm",)

    def __init__(self, name: str, kind: Any, attributes: dict[str, Any]) -> None:
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

    async def __aenter__(self) -> None:
        if self._cm is not None:
            try:
                await self._cm.__aenter__()
            except Exception:
                self._cm = None
        return None

    async def __aexit__(self, exc_type: Any, exc: Any, tb: Any) -> bool:
        if self._cm is not None:
            try:
                await self._cm.__aexit__(exc_type, exc, tb)
            except Exception:
                pass
        return False  # never suppress a user-callback exception


def safe_span(name: str, kind: Any = None, attributes: dict[str, Any] | None = None) -> _SafeSpan:
    """Open a zero-impact span. Always returns a usable (sync+async) context manager."""
    return _SafeSpan(name, kind, attributes or {})
