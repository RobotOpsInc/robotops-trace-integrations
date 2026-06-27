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

#include <gtest/gtest.h>

#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "example_interfaces/action/fibonacci.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"

#include "robotops_trace/trace.hpp"
#include "robotops_trace_rclcpp/robotops_trace_rclcpp.hpp"

namespace rtr = robotops::trace::rclcpp;
using Fibonacci = example_interfaces::action::Fibonacci;
using GoalHandleFib = rclcpp_action::ServerGoalHandle<Fibonacci>;
using namespace std::chrono_literals;

// ---------------------------------------------------------------------------
// Fixture: a fresh in-memory exporter + SDK per test, plus rclcpp lifecycle.
// ---------------------------------------------------------------------------
class RclcppTraceTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    exporter_ = std::make_shared<robotops::InMemorySpanExporter>();
    robotops::Config config;
    config.service_name = "robotops_trace_rclcpp_test";
    config.exporter = exporter_;
    robotops::init(std::move(config));
  }

  void TearDown() override
  {
    robotops::shutdown();
  }

  std::vector<robotops::SpanData> flush_spans()
  {
    robotops::force_flush(2s);
    return exporter_->spans();
  }

  static const robotops::SpanData * find_span(
    const std::vector<robotops::SpanData> & spans, const std::string & needle)
  {
    for (const auto & s : spans) {
      if (s.name.find(needle) != std::string::npos) {
        return &s;
      }
    }
    return nullptr;
  }

  static std::string attr(const robotops::SpanData & s, const std::string & key)
  {
    for (const auto & kv : s.attributes) {
      if (kv.first == key) {
        return kv.second.type() == robotops::AttributeValue::Type::String ?
               kv.second.string_value() : std::string{};
      }
    }
    return {};
  }

  std::shared_ptr<robotops::InMemorySpanExporter> exporter_;
};

// ===========================================================================
// (A) rclcpp_action: client -> server goal produces spans carrying the SAME
//     goal-UUID key on both sides (the deterministic ROB-427 join key), and the
//     client span nests under the caller's active context.
// ===========================================================================
TEST_F(RclcppTraceTest, ActionGoalUuidCorrelation)
{
  auto node = std::make_shared<rclcpp::Node>("action_test_node");
  std::vector<std::thread> server_threads;

  // --- traced action server ---
  auto handle_goal =
    [](const rclcpp_action::GoalUUID &, std::shared_ptr<const Fibonacci::Goal>) {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    };
  auto handle_cancel =
    [](const std::shared_ptr<GoalHandleFib>) {
      return rclcpp_action::CancelResponse::ACCEPT;
    };
  auto handle_accepted =
    [&server_threads](const std::shared_ptr<GoalHandleFib> goal_handle) {
      server_threads.emplace_back(
        [goal_handle]() {
          // Full-execution server span via the recommended in-execute helper.
          auto span = rtr::scoped_action_span<Fibonacci>(goal_handle, "/fibonacci");
          auto result = std::make_shared<Fibonacci::Result>();
          result->sequence = {0, 1, 1, 2, 3};
          goal_handle->succeed(result);
          span.span().set_attribute(rtr::keys::kRobotActionResult, "succeeded");
        });
    };

  auto server = rtr::create_traced_action_server<Fibonacci>(
    node, "/fibonacci", handle_goal, handle_cancel, handle_accepted);

  // --- traced action client ---
  auto client = rclcpp_action::create_client<Fibonacci>(node, "/fibonacci");
  ASSERT_TRUE(client->wait_for_action_server(5s)) << "action server not available";

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::promise<void> result_arrived;
  auto result_arrived_fut = result_arrived.get_future().share();

  Fibonacci::Goal goal;
  goal.order = 5;

  rclcpp_action::Client<Fibonacci>::SendGoalOptions opts;
  opts.result_callback =
    [&result_arrived](const rclcpp_action::ClientGoalHandle<Fibonacci>::WrappedResult &) {
      result_arrived.set_value();
    };

  // Capture the caller's active context: the client spans must nest under it.
  robotops::SpanContext caller_ctx;
  std::shared_future<rclcpp_action::ClientGoalHandle<Fibonacci>::SharedPtr> goal_future;
  {
    robotops::SpanGuard caller("client_caller");
    caller_ctx = caller.span().context();
    goal_future = rtr::send_traced_goal<Fibonacci>(client, goal, "/fibonacci", opts);
  }  // caller span closes; the captured context value survives in the callbacks

  ASSERT_EQ(
    executor.spin_until_future_complete(goal_future, 5s),
    rclcpp::FutureReturnCode::SUCCESS);
  auto goal_handle = goal_future.get();
  ASSERT_NE(goal_handle, nullptr) << "goal was rejected";

  ASSERT_EQ(
    executor.spin_until_future_complete(result_arrived_fut, 5s),
    rclcpp::FutureReturnCode::SUCCESS);

  for (auto & t : server_threads) {
    if (t.joinable()) {t.join();}
  }

  auto spans = flush_spans();
  ASSERT_FALSE(spans.empty());

  const auto * server_span = find_span(spans, "action.execute");
  const auto * client_span = find_span(spans, "action.result");
  ASSERT_NE(server_span, nullptr) << "missing server-side action.execute span";
  ASSERT_NE(client_span, nullptr) << "missing client-side action.result span";

  // The deterministic cross-process join key: identical goal UUID on both sides.
  const std::string server_goal_id = attr(*server_span, rtr::keys::kRobotActionGoalId);
  const std::string client_goal_id = attr(*client_span, rtr::keys::kRobotActionGoalId);
  EXPECT_FALSE(server_goal_id.empty());
  EXPECT_EQ(server_goal_id.size(), 36u) << "expected canonical 8-4-4-4-12 UUID";
  EXPECT_EQ(server_goal_id, client_goal_id)
    << "client and server must emit the SAME goal UUID (the ROB-427 join key)";

  EXPECT_EQ(attr(*client_span, rtr::keys::kRobotActionResult), "succeeded");
  EXPECT_EQ(server_span->kind, robotops::SpanKind::Server);
  EXPECT_EQ(client_span->kind, robotops::SpanKind::Client);

  // The client result span nests under the caller's active context.
  EXPECT_EQ(client_span->context.trace_id, caller_ctx.trace_id)
    << "client span should share the caller's trace";
  EXPECT_EQ(client_span->parent_span_id, caller_ctx.span_id)
    << "client span should be a child of the caller span";

  executor.remove_node(node);
}

// ===========================================================================
// (B) executor/callback hook: a wrapped subscription callback opens a span that
//     nests under the active thread-local context, and (C) carries best-effort
//     message content-correlation keys from rclcpp::MessageInfo.
// ===========================================================================
TEST_F(RclcppTraceTest, WrappedCallbackNestsAndCarriesContentKeys)
{
  bool callback_ran = false;

  auto traced_cb = rtr::traced_subscription<std::shared_ptr<const std_msgs::msg::String>>(
    "/chatter",
    [&callback_ran](std::shared_ptr<const std_msgs::msg::String>, const rclcpp::MessageInfo &) {
      callback_ran = true;
    });

  robotops::SpanContext parent_ctx;
  {
    // Simulate the active context an executor (or a parent span) would establish
    // on the callback thread.
    robotops::SpanGuard parent("subscriber_node");
    parent_ctx = parent.span().context();

    rmw_message_info_t rmw{};
    rmw.source_timestamp = 1234567890;
    rmw.publisher_gid.data[0] = 0xAB;
    rmw.publisher_gid.data[1] = 0xCD;
    rclcpp::MessageInfo info(rmw);

    traced_cb(std::make_shared<const std_msgs::msg::String>(), info);
  }

  EXPECT_TRUE(callback_ran);

  auto spans = flush_spans();
  const auto * cb_span = find_span(spans, "/chatter callback");
  ASSERT_NE(cb_span, nullptr) << "missing wrapped subscription callback span";

  // Nesting under the active context.
  EXPECT_EQ(cb_span->context.trace_id, parent_ctx.trace_id);
  EXPECT_EQ(cb_span->parent_span_id, parent_ctx.span_id);
  EXPECT_EQ(cb_span->kind, robotops::SpanKind::Consumer);

  // Content-correlation keys present.
  EXPECT_EQ(attr(*cb_span, rtr::keys::kRosTopic), "/chatter");
  EXPECT_FALSE(attr(*cb_span, rtr::keys::kRosPublisherGid).empty());
  bool has_source_ts = false;
  for (const auto & kv : cb_span->attributes) {
    if (kv.first == rtr::keys::kRosSourceTimestamp) {
      has_source_ts = true;
      EXPECT_EQ(kv.second.int_value(), 1234567890);
    }
  }
  EXPECT_TRUE(has_source_ts);
}

// ===========================================================================
// Canonical goal-UUID formatting contract (must match the rclpy side, ROB-423).
// ===========================================================================
TEST_F(RclcppTraceTest, GoalIdCanonicalFormat)
{
  rclcpp_action::GoalUUID uuid{
    {0xf4, 0x7a, 0xc1, 0x0b, 0x58, 0xcc, 0x43, 0x72,
      0xa5, 0x67, 0x0e, 0x02, 0xb2, 0xc3, 0xd4, 0x79}};
  EXPECT_EQ(rtr::goal_id_to_string(uuid), "f47ac10b-58cc-4372-a567-0e02b2c3d479");
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int rc = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return rc;
}
