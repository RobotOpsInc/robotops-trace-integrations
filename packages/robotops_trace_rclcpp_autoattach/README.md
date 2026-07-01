# robotops_trace_rclcpp_autoattach

**Issue:** [ROB-453](https://linear.app/robot-ops/issue/ROB-453) &nbsp;•&nbsp; **Status:** 🟡 first cut

Process-wide, **zero-code, fork-free** cross-process **goal-UUID** instrumentation
for stock `rclcpp_action` — the "L3" layer that stitches **stock Nav2**'s pipeline
(`mission → bt_navigator → planner/controller/behavior`) into **one connected
trace**.

```sh
export LD_PRELOAD=/opt/ros/$ROS_DISTRO/lib/librobotops_trace_rclcpp_autoattach.so
export ROBOTOPS_TRACE_AUTOINIT=1
ros2 launch nav2_bringup tb4_loopback_simulation.launch.py
# every action hop now emits a CLIENT + SERVER span carrying robot.action.goal_id;
# the robot-agent's TraceJoiner (ROB-427) joins them by that key.
```

## Mechanism

`rclcpp_action` puts the goal-UUID in **non-templated base classes**, so we don't
need a per-action-type interposer — the same technique as
[`robotops_trace_bt_cpp_autoattach`](../robotops_trace_bt_cpp_autoattach), applied
to `ServerBase`/`ClientBase`. We *define* the base-class methods in a preloaded
DSO (default visibility) and call the originals via `dlsym(RTLD_NEXT, …)`.

| Seam | Kind | What we do |
|---|---|---|
| `ServerBase::execute_goal_request_received(…, GoalInfo, …)` | non-virtual | OPEN a detached **SERVER** span (root), keyed by `GoalInfo.goal_id.uuid` |
| `ServerBase::publish_result(const GoalUUID&, …)` | non-virtual | CLOSE the span for that UUID |
| `ClientBase::generate_goal_id()` | virtual | capture the UUID (thread-local) |
| `ClientBase::send_goal_request(…)` | virtual | OPEN+CLOSE a **CLIENT** span parented to the current context |

- **Server spans** are opened at goal-received (one thread) and ended at result
  (another), so they use the SDK **`DetachedSpan`** primitive (ROB-443) keyed by
  UUID in a registry — never touching thread-local context. They're **roots**:
  the agent re-parents the consumer trace onto the matching producer.
- **Client spans** are `SpanKind::Client` + parented to `current_context()` (the
  mission span, or the BT action node executing when `bt_navigator` calls a
  sub-action) → the **producer**, staying in the caller's trace.
- Direction is carried by **`SpanKind`** (Client=producer, Server=consumer) — the
  exact signal the agent's `classify()` keys on. `robot.action.goal_id` uses the
  shared `goal_id_to_string()` (canonical RFC-4122) so it's byte-identical to the
  rclpy/rclcpp/bt sides.

## Zero robot impact

Every interposed body is `try`/caught + the SDK ops are `noexcept`; a tracing
fault never perturbs the action. Opt out with `ROBOTOPS_TRACE_RCLCPP_AUTOATTACH=0`
(or the SDK-wide `ROBOTOPS_TRACE_ENABLED=0`).

## Limitations

- **Client side is virtual.** `generate_goal_id`/`send_goal_request` are `virtual`
  — symbol interposition catches them only if `librclcpp_action.so`'s vtable uses
  symbolic relocations (i.e. built without `-Bsymbolic`, which jazzy's is). If a
  build binds them locally, the fallback is interposing the `rcl_action` C layer
  (`rcl_action_send_goal_request`), which is unconditionally interposable. The
  **server** side (non-virtual) is always reliable.
- **No action name at the base-class seam** — spans are named generically
  (`action.execute` / `action.goal`); `robot.action.name` is not set (the name
  lives in the templated derived class). The join only needs `goal_id`.
- **Server span status** — `publish_result` doesn't expose the terminal
  `GoalStatus` cleanly; the client-side result carries the authoritative status.
- Pinned to the `rclcpp_action` v-jazzy ABI (mangled base-class symbols). Verified
  against jazzy; humble is a follow-up.

## Building & testing

ROS 2 is not on the host — build in Docker. Validate against the headless Nav2
loopback sim + a running `robot-agent`: `LD_PRELOAD` the DSO into the bringup, run
a mission, and confirm the agent joins `bt_navigator`'s server span under the
mission and the planner/controller server spans under the BT action nodes.
