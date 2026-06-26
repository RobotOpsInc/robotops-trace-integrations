# robotops_trace_rclcpp

**Issue:** [ROB-422](https://linear.app/robotops/issue/ROB-422) &nbsp;•&nbsp; **Status:** 🟢 first cut

Opt-in, **fork-free** RobotOps Trace instrumentation for **rclcpp** — the
highest-leverage integration, since it covers every C++ ROS 2 node. Built on the
transport-agnostic [`robotops_trace_cpp`](../../) SDK core.

It does three things:

| # | Capability | Mechanism | Confidence |
|---|---|---|---|
| **A** | **rclcpp_action goal-UUID correlation** | wrap the public `create_server` callbacks + client `SendGoalOptions` | **solid / deterministic** |
| **B** | per-callback spans (executor) | wrap subscription/timer/service callbacks at creation | clean public hook, **opt-in only** |
| **C** | message content-correlation keys | read `rclcpp::MessageInfo` (publisher GID, source timestamp) | best-effort |

Everything wraps **public** rclcpp / rclcpp_action seams. **No rclcpp fork.**

```cpp
#include <robotops_trace_rclcpp/robotops_trace_rclcpp.hpp>
namespace rtr = robotops::trace::rclcpp;
```

---

## A. rclcpp_action — the deterministic goal-UUID key

This is the load-bearing feature. An action goal crosses a **process boundary**
(client process → server process), and RobotOps does **not** inject a W3C
`traceparent` into the goal message. Instead, the client and server each observe
the **same 16-byte goal UUID** for a goal, and both emit it as the
`robot.action.goal_id` span attribute. The correlation agent
([ROB-427](https://linear.app/robotops/issue/ROB-427)) **joins the client trace
to the server trace on that key.**

**The contract:** the goal UUID is rendered as the canonical RFC-4122
**8-4-4-4-12 lowercase hyphenated** string (e.g.
`f47ac10b-58cc-4372-a567-0e02b2c3d479`) via `rtr::goal_id_to_string()`. Both the
client and server here, **and the Python integration (ROB-423)**, MUST format it
identically or the join fails. (We deliberately do *not* use
`rclcpp_action::to_string()`, which renders 32 un-hyphenated hex chars.)

### Server

Drop-in replacement for `rclcpp_action::create_server` — wraps all three
callbacks, no body changes:

```cpp
auto server = rtr::create_traced_action_server<Fibonacci>(
  node, "/fibonacci", handle_goal, handle_cancel, handle_accepted);
```

This auto-emits Server spans on the goal decision (`action.goal_request`,
carrying the goal UUID + accept/reject), cancel, and accepted/dispatch.

For a span over the **whole goal execution** (the common case — work runs on a
thread the user spawns from `handle_accepted`), open one inside your `execute()`:

```cpp
void execute(const std::shared_ptr<GoalHandleFib> goal_handle) {
  auto span = rtr::scoped_action_span<Fibonacci>(goal_handle, "/fibonacci");
  // ... do the work ...
  goal_handle->succeed(result);
  span.span().set_attribute(rtr::keys::kRobotActionResult, "succeeded");
}
```

> **Why two entry points?** rclcpp runs a goal on whatever thread you drive, and
> `handle_accepted` typically returns immediately after spawning that thread. The
> drop-in wrapper's `action.accepted` span therefore covers *dispatch*, not the
> work; `scoped_action_span` is the one that covers the full unit of work. Both
> carry the goal UUID, so both correlate cross-process.

### Client

Wrap the `SendGoalOptions` (your own callbacks are preserved and run inside the
span scope):

```cpp
rclcpp_action::Client<Fibonacci>::SendGoalOptions opts;
opts.result_callback = [](const auto & r){ /* your code */ };

auto goal_future = rtr::send_traced_goal<Fibonacci>(client, goal, "/fibonacci", opts);
// or: client->async_send_goal(goal, rtr::trace_send_goal_options<Fibonacci>("/fibonacci", opts));
```

Emits a Client span on goal-response (accept/reject) and on the terminal result
(`action.result`, carrying the goal UUID + `robot.action.result`, status set from
the result code).

**Thread-safety note.** The SDK's span records are thread-confined (RAII on the
opening thread). An action is async — the result arrives on an executor thread,
not the caller's thread — so we never hold one span open across that boundary
(which would corrupt the thread-local stack). Instead each wrapper **captures the
caller's `SpanContext` synchronously** and **opens + closes a span entirely
inside each async callback**, parented to that captured context. The spans nest
correctly under the caller's trace and carry the goal UUID, on whatever thread
the callback runs.

---

## B. Per-callback spans (executor instrumentation)

```cpp
auto sub = node->create_subscription<std_msgs::msg::String>(
  "/chatter", 10,
  rtr::traced_callback("/chatter", robotops::SpanKind::Consumer, my_callback));

auto timer = node->create_wall_timer(1s, rtr::traced_callback("tick", my_timer_cb));
```

`traced_callback` wraps any callable (timer / subscription / service — args and
return value are perfectly forwarded) so each invocation opens a span nested
under the active thread-local context.

### Mechanism chosen, and its limits (honest)

jazzy exposes **no public seam to wrap *every* executor callback generically.**
`rclcpp::Executor`'s dispatch path (`execute_subscription`, `execute_timer`, …)
is not a virtual customization point, and a custom `Executor` subclass can't
inject a span around the user's callable without reaching into private machinery.
`ros2_tracing`/`tracetools` tracepoints are LTTng compile-time hooks, not a
span-emitting bridge into our SDK. So the clean, fork-free public hook is
**callback wrapping at creation time** (above).

**Limitation:** this is **opt-in per callback**, not automatic for the whole
process. Fully-automatic "a span per callback, zero code change" needs the
**LD_PRELOAD / auto-init layer ([ROB-421](https://linear.app/robotops/issue/ROB-421))**,
which will wrap the executor entry points process-wide and install a per-callback
`ScopedContext`. The wrappers here open their span under whatever context is
active on the executor thread, so they are **forward-compatible** with ROB-421.

---

## C. Message content-correlation keys (best-effort)

For a subscription with the 2-arg `void(MsgT, const rclcpp::MessageInfo&)`
signature, `traced_subscription` (or `record_message_info` directly) stamps the
publisher GID and source timestamp onto the span:

```cpp
auto cb = rtr::traced_subscription<std::shared_ptr<const std_msgs::msg::String>>(
  "/chatter", my_callback);
```

→ `ros.topic`, `ros.publisher_gid`, `ros.source_timestamp`. These let ROB-427
best-effort correlate a received message back to its publish, where the rmw layer
supplies them (DDS does). Best-effort: values depend on rmw support.

---

## Semantic conventions

Attribute keys are concept-level constants in
[`semantic_conventions.hpp`](include/robotops_trace_rclcpp/semantic_conventions.hpp)
(`robot.action.goal_id`, `robot.action.result`, `robot.action.name`,
`robot.action.goal_response`, `ros.topic`, `ros.publisher_gid`,
`ros.source_timestamp`). These migrate to the shared `robotops_trace_semconv`
package in [ROB-430](https://linear.app/robotops/issue/ROB-430)
(`robot.action.result` is already re-exported from there).

## Building & testing

ROS 2 is not on the host — build in Docker. This package builds against the SDK
core **from source** in a combined colcon workspace until
`ros-<distro>-robotops-trace-cpp` is published to apt:

```sh
# src/robotops-trace-cpp           <- the SDK core
# src/robotops-trace-integrations  <- this repo
colcon build --packages-up-to robotops_trace_rclcpp
colcon test  --packages-select  robotops_trace_rclcpp
```

The gtest suite ([`test/`](test/)) captures spans via the core's
`InMemorySpanExporter` and asserts: (1) an action client→server round trip emits
the **same** `robot.action.goal_id` on both sides; (2) a wrapped subscription
callback nests under the active context and carries the content keys; (3) the
goal-UUID canonical format.

> **CI status:** the integration CI resolves the core from apt, so it stays red
> until `robotops-trace-cpp` publishes to `apt.development`. The local combined
> colcon build above is the real verification for now.

## Distributes to

- **apt** as `ros-<distro>-robotops-trace-rclcpp` (one deb per ROS distro).
