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

#ifndef ROBOTOPS_TRACE_RCLCPP__CALLBACKS_HPP_
#define ROBOTOPS_TRACE_RCLCPP__CALLBACKS_HPP_

#include <cstdint>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "robotops_trace/trace.hpp"

#include "robotops_trace_rclcpp/identifiers.hpp"
#include "robotops_trace_rclcpp/semantic_conventions.hpp"

/// \file callbacks.hpp
/// \brief Per-callback span wrappers (executor instrumentation) + best-effort
/// message content-correlation keys.
///
/// MECHANISM (and why): jazzy exposes no public seam to wrap *every* executor
/// callback generically — `rclcpp::Executor`'s dispatch path
/// (`execute_subscription`, `execute_timer`, ...) is not a virtual customization
/// point, and a custom Executor subclass cannot inject a span around the user's
/// callable without reaching into private machinery. The clean, fork-free public
/// hook is therefore CALLBACK WRAPPING AT CREATION TIME: wrap the std::function
/// you hand to `create_subscription` / `create_wall_timer` / `create_service`.
///
/// LIMITATION (honest): this is OPT-IN per callback, not automatic for every
/// callback in the process. Fully-automatic "a span per callback with zero code
/// change" needs the LD_PRELOAD/auto-init layer (ROB-421) — which will wrap the
/// executor entry points process-wide. Until then, wrap the callbacks you care
/// about with the helpers below. A wrapped callback opens its span nested under
/// whatever context is active on the executor thread when it runs (e.g. a
/// `ScopedContext` an auto-init executor wrapper installs, or a content-derived
/// parent), so this is forward-compatible with ROB-421.

namespace robotops::trace::rclcpp
{

/// Wrap any callback so each invocation opens an Internal span named `span_name`
/// that nests under the active thread-local context. Works for any callback
/// signature (timer `void()`, subscription `void(Msg)`, service
/// `void(req, resp)`, ...): arguments and the return value are perfectly
/// forwarded.
template<typename Fn>
auto traced_callback(std::string span_name, Fn fn)
{
  return [span_name = std::move(span_name), fn = std::move(fn)](auto &&... args)
         -> decltype(fn(std::forward<decltype(args)>(args)...))
         {
           ::robotops::SpanGuard guard(span_name);
           return fn(std::forward<decltype(args)>(args)...);
         };
}

/// As `traced_callback`, but with an explicit span kind (e.g. Consumer for a
/// subscription, Server for a service).
template<typename Fn>
auto traced_callback(std::string span_name, ::robotops::SpanKind kind, Fn fn)
{
  return [span_name = std::move(span_name), kind, fn = std::move(fn)](auto &&... args)
         -> decltype(fn(std::forward<decltype(args)>(args)...))
         {
           ::robotops::SpanOptions opts;
           opts.kind = kind;
           ::robotops::SpanGuard guard(span_name, opts);
           return fn(std::forward<decltype(args)>(args)...);
         };
}

/// Stamp best-effort content-correlation keys (publisher GID + source
/// timestamp) from a subscription's `rclcpp::MessageInfo` onto a span. These let
/// the agent (ROB-427) best-effort correlate a received message back to its
/// publish, where the rmw layer supplies them (DDS does).
inline void record_message_info(
  ::robotops::Span span, const ::rclcpp::MessageInfo & message_info) noexcept
{
  const auto & rmw = message_info.get_rmw_message_info();
  span.set_attribute(
    keys::kRosSourceTimestamp, static_cast<std::int64_t>(rmw.source_timestamp));
  span.set_attribute(keys::kRosPublisherGid, gid_to_string(rmw.publisher_gid));
}

/// Wrap a subscription callback of the two-argument form
/// `void(MsgT, const rclcpp::MessageInfo&)` so each message opens a Consumer
/// span that ALSO carries the content-correlation keys (publisher GID, source
/// timestamp, topic). Use this when you constructed the subscription to receive
/// `MessageInfo`.
template<typename MsgT, typename Fn>
std::function<void(MsgT, const ::rclcpp::MessageInfo & info)>
traced_subscription(
  std::string topic, Fn fn)
{
  return [topic = std::move(topic), fn = std::move(fn)](
    MsgT msg, const ::rclcpp::MessageInfo & info)
         {
           ::robotops::SpanOptions opts;
           opts.kind = ::robotops::SpanKind::Consumer;
           ::robotops::SpanGuard guard(topic + " callback", opts);
           auto span = guard.span();
           span.set_attribute(keys::kRosTopic, topic);
           record_message_info(span, info);
           fn(std::move(msg), info);
         };
}

}  // namespace robotops::trace::rclcpp

#endif  // ROBOTOPS_TRACE_RCLCPP__CALLBACKS_HPP_
