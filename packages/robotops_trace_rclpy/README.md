# robotops_trace_rclpy

**Issue:** [ROB-423](https://linear.app/robotops/issue/ROB-423) &nbsp;•&nbsp; **Status:** 🟡 stub / scaffold

RobotOps Trace integration for **rclpy** — the only Python integration package.
Monkey-patches rclpy so trace context from the `robotops-trace` Python SDK core
propagates across executor callbacks and actions.

## Distributes to (two lanes)

- **apt** as `ros-<distro>-robotops-trace-rclpy` (built by `ament_python`).
- **PyPI** as `robotops-trace-rclpy` (OIDC Trusted Publishing) so pip-only
  Python users get it without apt — see the `pypi-publish` workflow.

## What's here now (scaffold)

- `ament_python` package: `package.xml` (build_type `ament_python`), `setup.py`,
  `setup.cfg`, ament resource marker.
- Stub module `robotops_trace_rclpy/__init__.py` exposing a no-op `install()`.

## What ROB-423 fills in

The actual monkey-patch of `rclpy.executors` / `rclpy.action` to capture and
restore trace context, wired to the `robotops_trace_semconv` keys.

> **Heads-up:** depends on the `robotops-trace` Python core (PyPI) and
> `robotops_trace_semconv`, neither of which is published yet. Dependency
> resolution / the PyPI publish will fail until the cores ship. Expected.
