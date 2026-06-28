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

"""RobotOps Trace integration for rclpy (ROB-423) — Python node parity.

Monkey-patches stock ``rclpy`` so existing Python ROS 2 nodes get traced with no
code changes and no fork. Just import it::

    import robotops_trace_rclpy   # auto-installs the patches on import

    import robotops
    robotops.init(service_name="my_node")   # bring up the SDK as usual

What gets traced (parity with the C++ ``rclcpp`` integration):
  * subscription / timer / service / service-client callbacks → per-callback
    spans tagged with ``robot.callback.type``;
  * actions → ``robot.action.goal_id`` (the canonical RFC-4122 8-4-4-4-12
    lowercase UUID, byte-identical to the rclcpp integration) emitted CLIENT-side
    on ``send_goal`` and SERVER-side around ``execute_callback`` — so the ROB-427
    agent join stitches a Python client to a C++ server (or vice versa).

The patch auto-installs on import. Set ``ROBOTOPS_TRACE_RCLPY_AUTOPATCH=0`` to
opt out and call :func:`install` yourself. Patching and the wrappers are
idempotent and zero-impact: an instrumentation error can never break a node, and
if the SDK is uninitialised the wrappers are transparent pass-throughs.
"""

from __future__ import annotations

import os

from ._identifiers import format_goal_id, goal_id_bytes
from ._patch import install, is_installed, uninstall

__version__ = "0.2.0"

__all__ = [
    "__version__",
    "install",
    "uninstall",
    "is_installed",
    "format_goal_id",
    "goal_id_bytes",
]


def _autopatch_enabled() -> bool:
    return os.environ.get("ROBOTOPS_TRACE_RCLPY_AUTOPATCH", "1").lower() not in (
        "0",
        "false",
        "no",
    )


# Auto-install on import (the documented "just import it" UX). install() is
# itself best-effort and idempotent, so this never raises into the importer.
if _autopatch_enabled():
    install()
