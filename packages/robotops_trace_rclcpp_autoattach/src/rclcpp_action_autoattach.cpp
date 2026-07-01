// Copyright 2026 Robot Ops Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// ROB-453 — LD_PRELOAD auto-attach for stock rclcpp_action (the cross-process
// goal-UUID join, "L3").
//
// Interposes the NON-templated base-class seams of rclcpp_action that carry the
// goal-UUID, so STOCK Nav2 (bt_navigator, planner/controller/behavior servers)
// emits CLIENT + SERVER spans keyed by robot.action.goal_id — with zero code
// changes and no fork. The robot-agent's TraceJoiner (ROB-427) then stitches the
// client trace to the server trace by that key, rendering the whole
// mission -> bt_navigator -> planner/controller pipeline as ONE connected trace.
//
//   LD_PRELOAD=.../librobotops_trace_rclcpp_autoattach.so ros2 launch nav2_bringup ...
//
// SEAMS (mangled names from `nm -D librclcpp_action.so`, jazzy):
//   * SERVER (non-virtual -> reliably interposable):
//       ServerBase::execute_goal_request_received(ret, GoalInfo, req_id, msg)
//         -> OPEN a detached SERVER span (root; the agent re-parents it onto the
//            matching CLIENT span), keyed in a UUID registry.
//       ServerBase::publish_result(const GoalUUID&, msg)
//         -> CLOSE the span for that UUID.
//   * CLIENT (VIRTUAL -> interposes only if librclcpp_action's vtable uses
//     symbolic relocations, i.e. not -Bsymbolic; harmless dead code otherwise):
//       ClientBase::generate_goal_id() -> capture the UUID into a thread-local.
//       ClientBase::send_goal_request(req, cb) -> OPEN+CLOSE a CLIENT span
//         parented to the current context (so it nests under the mission / the BT
//         action node), carrying the captured UUID. This is the "producer".
//
// Direction is carried by SpanKind (Client=producer, Server=consumer) — the exact
// signal the agent's classify() keys on. goal_id is formatted with the shared
// rclcpp goal_id_to_string() so it is byte-identical to the rclpy/rclcpp/bt sides.
//
// ZERO ROBOT IMPACT: every body is try/caught + the SDK ops are noexcept; a
// tracing fault never perturbs the action. Opt out with
// ROBOTOPS_TRACE_RCLCPP_AUTOATTACH=0 (or the SDK-wide ROBOTOPS_TRACE_ENABLED=0).

#include <dlfcn.h>

#include <array>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include <rclcpp_action/rclcpp_action.hpp>  // NOLINT(build/include_order): ServerBase/ClientBase/GoalUUID

#include <robotops_trace_rclcpp/identifiers.hpp>
#include <robotops_trace_semconv/semconv.hpp>
#include <robotops_trace/trace.hpp>

namespace
{

namespace semconv = robotops::trace::semconv;

bool env_off(const char * name)
{
  const char * v = std::getenv(name);
  if (v == nullptr || *v == '\0') {
    return false;
  }
  return std::strcmp(v, "0") == 0 || std::strcmp(v, "false") == 0 ||
         std::strcmp(v, "off") == 0 || std::strcmp(v, "FALSE") == 0;
}

bool enabled()
{
  static const bool on =
    !env_off("ROBOTOPS_TRACE_RCLCPP_AUTOATTACH") && !env_off("ROBOTOPS_TRACE_ENABLED");
  return on;
}

void ensure_init()
{
  static std::once_flag once;
  std::call_once(once, [] {robotops::init();});
}

std::string format_uuid(const rclcpp_action::GoalUUID & uuid)
{
  return robotops::trace::rclcpp::goal_id_to_string(uuid);
}

// Hash for GoalUUID (std::array<uint8_t,16>) so it can key an unordered_map.
struct UuidHash
{
  std::size_t operator()(const rclcpp_action::GoalUUID & u) const noexcept
  {
    std::size_t h = 1469598103934665603ULL;  // FNV-1a
    for (auto b : u) {
      h = (h ^ b) * 1099511628211ULL;
    }
    return h;
  }
};

// Server spans live from goal-received (one thread) to result (another) → keyed
// by UUID with the detached-span primitive (never touches thread-local context).
std::mutex g_mu;
std::unordered_map<rclcpp_action::GoalUUID, robotops::DetachedSpan, UuidHash> g_server_spans;

// Client: generate_goal_id() runs immediately before send_goal_request() on the
// same thread → hand the UUID across via a thread-local.
thread_local rclcpp_action::GoalUUID g_client_uuid{};
thread_local bool g_client_uuid_valid = false;

void open_server_span(const rclcpp_action::GoalUUID & uuid) noexcept
{
  if (!enabled()) {
    return;
  }
  try {
    ensure_init();
    const std::string gid = format_uuid(uuid);
    // Root (no parent) → the agent re-parents this consumer trace onto the
    // producer (client) span that shares the same goal_id.
    robotops::SpanOptions opts;
    opts.kind = robotops::SpanKind::Server;
    robotops::DetachedSpan span = robotops::start_detached_span("action.execute", opts);
    span.set_attribute(semconv::kRobotActionGoalId, gid);
    span.set_attribute(semconv::kRobotCallbackType, semconv::callback_type::kAction);
    std::lock_guard<std::mutex> lock(g_mu);
    g_server_spans[uuid] = std::move(span);
  } catch (...) {
  }
}

void close_server_span(const rclcpp_action::GoalUUID & uuid) noexcept
{
  try {
    std::lock_guard<std::mutex> lock(g_mu);
    auto it = g_server_spans.find(uuid);
    if (it != g_server_spans.end()) {
      it->second.end();
      g_server_spans.erase(it);
    }
  } catch (...) {
  }
}

void emit_client_span() noexcept
{
  if (!enabled() || !g_client_uuid_valid) {
    g_client_uuid_valid = false;
    return;
  }
  try {
    ensure_init();
    const std::string gid = format_uuid(g_client_uuid);
    // Parent to the current context on this thread (the mission span, or the BT
    // action node that is executing when bt_navigator calls a sub-action) → this
    // is the producer, and it stays in the caller's trace.
    robotops::SpanContext parent = robotops::current_context();
    robotops::SpanOptions opts;
    opts.kind = robotops::SpanKind::Client;
    if (parent.valid()) {
      opts.parent = &parent;
    }
    robotops::DetachedSpan span = robotops::start_detached_span("action.goal", opts);
    span.set_attribute(semconv::kRobotActionGoalId, gid);
    span.set_attribute(semconv::kRobotCallbackType, semconv::callback_type::kClient);
    span.end();  // short producer span; the join only needs the key + parentage
  } catch (...) {
  }
  g_client_uuid_valid = false;
}

// --- resolved real symbols (Itanium ABI: `this` is the explicit first arg) ---
using RealExecGRR = void (*)(
  rclcpp_action::ServerBase *, rcl_ret_t, rcl_action_goal_info_t, rmw_request_id_t,
  std::shared_ptr<void>);
using RealPublishResult = void (*)(
  rclcpp_action::ServerBase *, const rclcpp_action::GoalUUID &, std::shared_ptr<void>);
using RealGenId = rclcpp_action::GoalUUID (*)(rclcpp_action::ClientBase *);
// ClientBase::ResponseCallback is a *protected* alias, so spell out the underlying
// type (identical type → identical mangled symbol) to name it at namespace scope.
// jazzy uncrustify wants `void(...)`, humble uncrustify wants `void (...)` — the two
// disagree and whitespace doesn't affect the mangled name, so freeze this line.
// *INDENT-OFF*
using RealSendGoalReq = void (*)(
  rclcpp_action::ClientBase *, std::shared_ptr<void>,
  std::function<void(std::shared_ptr<void>)>);
// *INDENT-ON*

constexpr const char * kExecGRR =
  "_ZN13rclcpp_action10ServerBase29execute_goal_request_receivedEi26action_msgs__"
  "msg__GoalInfo16rmw_request_id_sSt10shared_ptrIvE";
constexpr const char * kPublishResult =
  "_ZN13rclcpp_action10ServerBase14publish_resultERKSt5arrayIhLm16EESt10shared_ptrIvE";
constexpr const char * kGenId = "_ZN13rclcpp_action10ClientBase16generate_goal_idEv";
constexpr const char * kSendGoalReq =
  "_ZN13rclcpp_action10ClientBase17send_goal_requestESt10shared_ptrIvESt8functionIFvS2_EE";

}  // namespace

// ---------------------------------------------------------------------------
// Interposed definitions (default visibility → override librclcpp_action.so).
// ---------------------------------------------------------------------------
namespace rclcpp_action
{

void ServerBase::execute_goal_request_received(
  rcl_ret_t ret, rcl_action_goal_info_t goal_info, rmw_request_id_t request_header,
  std::shared_ptr<void> message)
{
  static const auto real = reinterpret_cast<RealExecGRR>(::dlsym(RTLD_NEXT, kExecGRR));
  // The `goal_info` argument is NOT yet populated at this seam (empty UUID). The
  // goal-UUID is the first field (offset 0) of the SendGoal request `message` —
  // rosidl's universal action layout is `{ UUID goal_id; Goal goal; }`, and the
  // request object is at least 16 bytes — so read the 16 bytes there.
  GoalUUID uuid{};
  bool have_uuid = false;
  if (message) {
    std::memcpy(uuid.data(), message.get(), uuid.size());
    have_uuid = true;
  }
  if (real) {
    real(this, ret, goal_info, request_header, message);
  }
  if (have_uuid) {
    open_server_span(uuid);
  }
}

void ServerBase::publish_result(const GoalUUID & uuid, std::shared_ptr<void> result_msg)
{
  static const auto real = reinterpret_cast<RealPublishResult>(::dlsym(RTLD_NEXT, kPublishResult));
  if (real) {
    real(this, uuid, result_msg);
  }
  close_server_span(uuid);
}

GoalUUID ClientBase::generate_goal_id()
{
  static const auto real = reinterpret_cast<RealGenId>(::dlsym(RTLD_NEXT, kGenId));
  GoalUUID id = real ? real(this) : GoalUUID{};
  try {
    g_client_uuid = id;
    g_client_uuid_valid = true;
  } catch (...) {
  }
  return id;
}

void ClientBase::send_goal_request(
  std::shared_ptr<void> request, std::function<void(std::shared_ptr<void>)> callback)
{
  static const auto real = reinterpret_cast<RealSendGoalReq>(::dlsym(RTLD_NEXT, kSendGoalReq));
  emit_client_span();  // consumes the thread-local UUID captured by generate_goal_id
  if (real) {
    real(this, request, callback);
  }
}

}  // namespace rclcpp_action
