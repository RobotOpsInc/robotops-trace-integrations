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

#include <array>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp/bt_factory.h"

#include "robotops_trace/trace.hpp"
#include "robotops_trace_bt_cpp/robotops_trace_bt_cpp.hpp"

namespace rtb = robotops::trace::bt;
using namespace std::chrono_literals;

// ---------------------------------------------------------------------------
// Test nodes.
// ---------------------------------------------------------------------------

// An ASYNC action: RUNNING for a couple of ticks, then SUCCESS. This is the node
// that spans multiple tree ticks (Nav2's NavigateToPose, MoveIt Pro's planning
// actions behave like this). Its single execution span must therefore stay open
// across ticks and still nest correctly under its parent.
class AsyncCounterAction : public BT::StatefulActionNode
{
public:
  AsyncCounterAction(const std::string & name, const BT::NodeConfig & config)
  : BT::StatefulActionNode(name, config) {}

  static BT::PortsList providedPorts() {return {};}

  BT::NodeStatus onStart() override
  {
    running_ticks_ = 0;
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    // Two RUNNING ticks (onRunning calls) before SUCCESS => the node is alive
    // across >=3 total ticks of the tree.
    return (++running_ticks_ >= 2) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
  }

  void onHalted() override {}

private:
  int running_ticks_{0};
};

// ---------------------------------------------------------------------------
// Fixture: a fresh in-memory exporter + SDK per test.
// ---------------------------------------------------------------------------
class BtTraceTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    exporter_ = std::make_shared<robotops::InMemorySpanExporter>();
    robotops::Config config;
    config.service_name = "robotops_trace_bt_cpp_test";
    config.exporter = exporter_;
    robotops::init(std::move(config));

    factory_.registerNodeType<AsyncCounterAction>("AsyncCounter");
    factory_.registerSimpleAction(
      "SyncOk", [](BT::TreeNode &) {return BT::NodeStatus::SUCCESS;});
    factory_.registerSimpleAction(
      "SyncFail", [](BT::TreeNode &) {return BT::NodeStatus::FAILURE;});
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

  static std::size_t count_named(
    const std::vector<robotops::SpanData> & spans, const std::string & name)
  {
    std::size_t n = 0;
    for (const auto & s : spans) {
      if (s.name == name) {++n;}
    }
    return n;
  }

  static const robotops::SpanData * find_named(
    const std::vector<robotops::SpanData> & spans, const std::string & name)
  {
    for (const auto & s : spans) {
      if (s.name == name) {return &s;}
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
  BT::BehaviorTreeFactory factory_;
};

// ===========================================================================
// (A) one span per EXECUTED node (NOT per tick), and
// (B) children nest under the Sequence (tree-structured parenting), and
// (D) the async node's span stays open across ticks yet still nests correctly.
// ===========================================================================
TEST_F(BtTraceTest, PerNodeSpansNestedByTreeStructureAcrossTicks)
{
  static constexpr const char * kXml =
    R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="MainTree">
        <Sequence name="root_seq">
          <AsyncCounter name="async_leaf"/>
          <SyncOk name="sync_leaf"/>
        </Sequence>
      </BehaviorTree>
    </root>)";

  BT::Tree tree = factory_.createTreeFromText(kXml);
  rtb::TreeTracer tracer(tree);   // attach via the public StatusChangeLogger seam

  // --- Drive the tree one tick at a time so we can observe cross-tick state ---
  int ticks = 0;
  BT::NodeStatus status = BT::NodeStatus::IDLE;

  status = tree.tickExactlyOnce();
  ++ticks;
  // After the FIRST tick the async leaf is RUNNING => its span is still OPEN,
  // so it must NOT have been exported yet (proves "one span per execution",
  // not per tick, and that the span survives across ticks).
  EXPECT_EQ(status, BT::NodeStatus::RUNNING);
  EXPECT_EQ(count_named(flush_spans(), "async_leaf"), 0u)
    << "async leaf span exported while still RUNNING (would be per-tick, not per-execution)";

  while (status == BT::NodeStatus::RUNNING && ticks < 100) {
    status = tree.tickExactlyOnce();
    ++ticks;
  }
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
  EXPECT_GE(ticks, 3) << "async node should span multiple ticks";

  auto spans = flush_spans();

  // (A) exactly one span per executed node — no per-tick flood.
  EXPECT_EQ(count_named(spans, "root_seq"), 1u);
  EXPECT_EQ(count_named(spans, "async_leaf"), 1u);
  EXPECT_EQ(count_named(spans, "sync_leaf"), 1u);
  EXPECT_EQ(spans.size(), 3u) << "expected exactly 3 execution spans for 3 executed nodes";

  const auto * seq = find_named(spans, "root_seq");
  const auto * async_leaf = find_named(spans, "async_leaf");
  const auto * sync_leaf = find_named(spans, "sync_leaf");
  ASSERT_NE(seq, nullptr);
  ASSERT_NE(async_leaf, nullptr);
  ASSERT_NE(sync_leaf, nullptr);

  // (B)+(D) both leaves nest under the Sequence's span (tree structure), and the
  // async leaf does so even though its span opened a tick before it closed.
  EXPECT_EQ(async_leaf->parent_span_id, seq->context.span_id)
    << "async leaf must be a child of the Sequence (tree-structured nesting)";
  EXPECT_EQ(sync_leaf->parent_span_id, seq->context.span_id)
    << "sync leaf must be a child of the Sequence";
  EXPECT_EQ(async_leaf->context.trace_id, seq->context.trace_id);
  EXPECT_EQ(sync_leaf->context.trace_id, seq->context.trace_id);

  // The Sequence is the tree root => its span is a root span (no parent).
  const std::array<std::uint8_t, 8> kZero{};
  EXPECT_EQ(seq->parent_span_id, kZero) << "the tree root span should have no parent";

  // Attributes: span name == node name; bt.node_type == registrationName;
  // terminal bt.status; and semconv robot.component.name carries the node name.
  EXPECT_EQ(attr(*async_leaf, rtb::keys::kBtNodeType), "AsyncCounter");
  EXPECT_EQ(attr(*seq, rtb::keys::kBtNodeType), "Sequence");
  EXPECT_EQ(attr(*async_leaf, rtb::keys::kBtStatus), "success");
  EXPECT_EQ(attr(*async_leaf, rtb::keys::kRobotComponentName), "async_leaf");

  // Successful nodes carry an Ok span status.
  EXPECT_EQ(async_leaf->status_code, robotops::StatusCode::Ok);
  EXPECT_EQ(sync_leaf->status_code, robotops::StatusCode::Ok);
  EXPECT_EQ(seq->status_code, robotops::StatusCode::Ok);
}

// ===========================================================================
// (C) a FAILURE node yields an Error-status span (and the failing Sequence too).
// ===========================================================================
TEST_F(BtTraceTest, FailureNodeProducesErrorSpan)
{
  static constexpr const char * kXml =
    R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="MainTree">
        <Sequence name="root_seq">
          <SyncFail name="fail_leaf"/>
        </Sequence>
      </BehaviorTree>
    </root>)";

  BT::Tree tree = factory_.createTreeFromText(kXml);
  rtb::TreeTracer tracer(tree);

  const BT::NodeStatus status = tree.tickWhileRunning();
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);

  auto spans = flush_spans();
  const auto * fail_leaf = find_named(spans, "fail_leaf");
  ASSERT_NE(fail_leaf, nullptr);

  EXPECT_EQ(fail_leaf->status_code, robotops::StatusCode::Error)
    << "a FAILURE node must map to a span StatusCode::Error";
  EXPECT_EQ(attr(*fail_leaf, rtb::keys::kBtStatus), "failure");
  EXPECT_EQ(count_named(spans, "fail_leaf"), 1u);

  // The Sequence propagates FAILURE, so its span is also an error.
  const auto * seq = find_named(spans, "root_seq");
  ASSERT_NE(seq, nullptr);
  EXPECT_EQ(seq->status_code, robotops::StatusCode::Error);
}

// ===========================================================================
// Zero-robot-impact: attaching a tracer while the SDK is DISABLED must not throw
// and must not perturb the tick result.
// ===========================================================================
TEST_F(BtTraceTest, DisabledSdkIsANoOpAndDoesNotPerturbTick)
{
  robotops::shutdown();
  robotops::Config config;
  config.service_name = "disabled";
  config.enabled = false;            // kill switch
  config.exporter = exporter_;
  robotops::init(std::move(config));

  static constexpr const char * kXml =
    R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="MainTree">
        <Sequence name="root_seq">
          <AsyncCounter name="async_leaf"/>
          <SyncOk name="sync_leaf"/>
        </Sequence>
      </BehaviorTree>
    </root>)";

  BT::Tree tree = factory_.createTreeFromText(kXml);
  rtb::TreeTracer tracer(tree);

  const BT::NodeStatus status = tree.tickWhileRunning();
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS) << "tick must succeed regardless of tracing";
  EXPECT_TRUE(flush_spans().empty()) << "a disabled SDK must emit no spans";
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
