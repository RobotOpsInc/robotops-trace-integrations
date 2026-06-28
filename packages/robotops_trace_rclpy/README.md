# robotops_trace_rclpy

**Issue:** [ROB-423](https://linear.app/robotops/issue/ROB-423) &nbsp;•&nbsp; **Status:** 🟢 first cut

Opt-in, **fork-free** RobotOps Trace instrumentation for **rclpy** — the Python
node parity to the C++ [`robotops_trace_rclcpp`](../robotops_trace_rclcpp/)
integration. It **monkey-patches stock `rclpy` at import**, so existing Python
ROS 2 nodes get traced with **no code changes**. Built on the
[`robotops-trace`](https://github.com/RobotOpsInc/robotops-trace-python) Python
SDK core and the shared [`robotops_trace_semconv`](../robotops_trace_semconv/)
keys.

```python
import robotops_trace_rclpy   # auto-installs the patches on import

import robotops
robotops.init(service_name="my_node")   # bring up the SDK as usual
# ... your normal rclpy node code is now traced ...
```

Set `ROBOTOPS_TRACE_RCLPY_AUTOPATCH=0` to opt out of auto-install and call
`robotops_trace_rclpy.install()` yourself.

| # | Capability | Mechanism | Confidence |
|---|---|---|---|
| **A** | **action goal-UUID correlation** (cross-language) | wrap `ActionClient.send_goal_async` + `ActionServer` `execute_callback` | **solid / deterministic** |
| **B** | per-callback spans (executor) | patch `Node.create_subscription`/`create_timer`/`create_service`/`create_client` | clean public hook, **auto on import** |
| **C** | message content-correlation keys | `ros.publisher_gid` / `ros.source_timestamp` from `MessageInfo` | **not available in rclpy** (see limitations) |

Everything patches **public** rclpy / rclpy.action seams. **No rclpy fork.**

---

## A. Actions — the deterministic, cross-language goal-UUID key

This is the load-bearing feature. An action goal crosses a **process boundary**
(client process → server process), and RobotOps does **not** inject a W3C
`traceparent` into the goal message. Instead, the client and server each observe
the **same 16-byte goal UUID** for a goal, and both emit it as the
`robot.action.goal_id` span attribute. The correlation agent
([ROB-427](https://linear.app/robotops/issue/ROB-427)) **joins the client trace
to the server trace on that key.**

**The cross-language contract:** the goal UUID is rendered as the canonical
RFC-4122 **8-4-4-4-12 lowercase hyphenated** string (e.g.
`f47ac10b-58cc-4372-a567-0e02b2c3d479`) — **byte-identical** to the rclcpp
integration's `goal_id_to_string()`. rclcpp builds it by hand (lowercase-hex the
16 bytes, then hyphenate); here, Python's `uuid.UUID(bytes=...)` renders exactly
that form from the same 16 bytes. So a **Python client → C++ server** (or C++
client → Python server) join is deterministic.

- **Client side** (`ActionClient.send_goal_async`, and the sync `send_goal` which
  delegates to it): the goal UUID is materialised up-front and a **CLIENT**-kind
  span `"<action> action.goal"` is opened carrying `robot.action.goal_id` and
  `robot.action.name`.
- **Server side** (`ActionServer` `execute_callback`): a **SERVER**-kind span
  `"<action> action.execute"` is opened over the whole execution, carrying the
  same `robot.action.goal_id` (read from `goal_handle.goal_id`). This is the
  parity of rclcpp's `scoped_action_span`. Both sync and `async def` execute
  callbacks are supported (the span is held across the `await`).

The CLIENT/SERVER span kinds match the rclcpp ROB-427 direction signal.

## B. Per-callback spans (executor instrumentation)

Patching `Node.create_*` wraps the user callback at creation time — the clean,
fork-free public hook (rclpy's executor dispatch is not a public customization
point). Each invocation opens a span tagged with `robot.callback.type`:

| Patched seam | Span | kind | `robot.callback.type` |
|---|---|---|---|
| `create_subscription` | `"<topic> subscription"` | consumer | `subscription` |
| `create_timer` | `"timer"` | internal | `timer` |
| `create_service` | `"<service> service"` | server | `service` |
| `create_client` → `call_async`/`call` | `"<service> call"` | client | `client` |

Unlike the rclcpp integration (opt-in per callback), this is **automatic on
import** for every node in the process — the monkey-patch covers all callbacks
created through the standard `Node.create_*` API.

Intra-process nesting (a callback span nesting under whatever span is already
active) is delegated to the SDK's contextvar-based current-span tracking.

## C. Message content-correlation keys — limitation

The rclcpp integration stamps `ros.publisher_gid` / `ros.source_timestamp` from
`rclcpp::MessageInfo`. **rclpy does not readily expose this:** the rclpy executor
takes the message and invokes the subscription callback with **only the message**
(`callback(msg)`); the `rmw_message_info` (publisher GID, source timestamp) is
discarded before the callback runs and is not surfaced through any public
per-callback seam. There is therefore **no fork-free hook** to attach these keys
to a subscription span in rclpy today. We emit `ros.topic` and `ros.message.type`
(available at subscription creation) and document the gap. Closing it would
require either a rclpy change to forward `MessageInfo` to callbacks, or the
process-wide auto-init layer (ROB-421) reaching the rmw take path.

## Semantic conventions

All attribute keys come from the shared `robotops_trace_semconv` Python module
(the byte-for-byte mirror of the C++ header) — `robot.action.goal_id`,
`robot.action.name`, `robot.callback.type` (+ its `subscription`/`timer`/
`service`/`client` values), `ros.topic`, `ros.message.type`. Sharing the
dictionary is what guarantees the emitted strings are identical to the rclcpp
integration and the agent/ROSQL vocabulary.

## Zero robot impact + idempotency

- **Never raises into user code.** Every patch and wrapper is defensively
  guarded; an instrumentation error (building attributes, the SDK raising, even a
  span `__enter__`/`__exit__` failing) is swallowed. A **user-callback** error,
  by contrast, propagates untouched — we never suppress it.
- **Transparent when the SDK is off.** Spans are opened through `robotops.span`,
  which is a no-op when the SDK is uninitialised/disabled, so the wrappers add
  nothing.
- **Idempotent patch.** Importing/installing twice does not double-wrap — each
  patched attribute and wrapped callback carries a sentinel. `uninstall()`
  restores stock rclpy (used by tests).
- **Forward-compatible span kind.** Kind is passed via `kind=`; on an SDK build
  that doesn't accept it, the wrapper transparently retries without it.

## Building & testing

ROS 2 is not on the host — build/test in `ros:jazzy` Docker. The integration
depends on the Python SDK core and the semconv mirror, **neither yet published**;
install them from source into the test container:

```sh
pip install --no-deps --break-system-packages \
    ../robotops-trace-python ../robotops_trace_semconv
colcon build  --packages-select robotops_trace_rclpy
colcon test   --packages-select robotops_trace_rclpy --event-handlers console_direct+
```

The pytest suite ([`test/`](test/)) asserts: the canonical goal-UUID format
matches the rclcpp literal; the patch is idempotent; wrapped subscription / timer
/ service callbacks open the right spans and still run; an instrumentation error
never breaks the callback; and a **real** `example_interfaces/Fibonacci` action
client→server round trip emits the **same** canonical `robot.action.goal_id` on
both sides (the deterministic ROB-427 join key).

**Real vs mocked:** rclpy is **real** (real `Node`, real action round trip over
the middleware). The span **sink** is mocked: the `robotops` Python SDK core is
currently a no-op scaffold (its `span()` yields `None` and it ships **no**
in-memory exporter), so the tests record what the integration asks the SDK to
open by monkeypatching `robotops.span`. When the real SDK + exporter land
(ROB-420) the assertions become end-to-end unchanged.

> **CI status:** the C++ core resolves from `apt.development`, but the Python SDK
> core (`robotops-trace`) is not on PyPI yet; it is only an `exec_depend` of this
> ament_python package, so colcon builds it without the core (the Dockerfile
> rosdep step `--skip-keys` it). The colcon build+test above is the real
> verification for now.

## Distributes to (two lanes)

- **apt** as `ros-<distro>-robotops-trace-rclpy` (built by `ament_python`).
- **PyPI** as `robotops-trace-rclpy` (so pip-only Python users get it without apt).
