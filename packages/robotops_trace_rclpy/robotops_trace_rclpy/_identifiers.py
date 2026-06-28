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

"""Canonical, cross-language string formatting for action correlation keys.

The formatting here is the *contract*: every RobotOps integration that emits the
action goal UUID (the C++ ``rclcpp`` integration's ``goal_id_to_string`` and this
``rclpy`` integration) MUST format it identically, or the correlation agent
(ROB-427) cannot join the client trace to the server trace across the process
*and language* boundary.

The canonical form is the RFC-4122 **8-4-4-4-12 lowercase hyphenated** string
(e.g. ``f47ac10b-58cc-4372-a567-0e02b2c3d479``). ``rclcpp`` builds it by hand
(lowercase-hex the 16 bytes, then hyphenate); Python's :class:`uuid.UUID` renders
exactly that form from the same 16 bytes, so the two are byte-for-byte identical.
"""

from __future__ import annotations

import uuid as _uuid
from typing import Any

__all__ = ["format_goal_id", "goal_id_bytes"]


def goal_id_bytes(goal_id: Any) -> bytes:
    """Coerce an rclpy goal identifier to its raw 16 bytes.

    Accepts any of the shapes a goal UUID arrives in:
      * a ``unique_identifier_msgs/UUID`` message (has a ``.uuid`` ``uint8[16]``
        field, which rclpy materialises as a ``numpy.ndarray``),
      * a raw ``bytes`` / ``bytearray`` / list / numpy array of 16 ``uint8``.

    Raises:
        ValueError: if the result is not exactly 16 bytes.
    """
    raw = getattr(goal_id, "uuid", goal_id)
    data = bytes(bytearray(raw))
    if len(data) != 16:
        raise ValueError(f"goal UUID must be 16 bytes, got {len(data)}")
    return data


def format_goal_id(goal_id: Any) -> str:
    """Format an rclpy goal UUID as the canonical RFC-4122 8-4-4-4-12 string.

    Identical to the rclcpp integration's ``goal_id_to_string`` for the same 16
    bytes — that identity is what makes the cross-language action hop
    deterministic for the ROB-427 join.
    """
    return str(_uuid.UUID(bytes=goal_id_bytes(goal_id)))
