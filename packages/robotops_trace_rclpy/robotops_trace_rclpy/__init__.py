# Copyright 2025 Robot Ops Inc.
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

"""RobotOps Trace integration for rclpy (ROB-423) — STUB.

Will monkey-patch rclpy so trace context (from the ``robotops-trace`` Python SDK
core) propagates across executor callbacks and rclpy actions, using the keys
from ``robotops_trace_semconv``. Ships to both apt (ament_python) and PyPI.

This is a SCAFFOLD: ``install()`` is a no-op placeholder. Real patching lands in
ROB-423.
"""

__version__ = "0.1.0"


def install() -> None:
    """Install the rclpy trace hooks. STUB — no-op until ROB-423.

    The real implementation will monkey-patch the rclpy executor + action
    client/server to capture and restore trace context. For now this only
    exists so the package import surface is stable.
    """
    # TODO(ROB-423): patch rclpy.executors / rclpy.action to propagate context.
    return None


__all__ = ["install", "__version__"]
