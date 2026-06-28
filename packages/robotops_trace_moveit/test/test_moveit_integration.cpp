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
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "robotops_trace/trace.hpp"
#include "robotops_trace_moveit/robotops_trace_moveit.hpp"

namespace rtm = robotops::trace::moveit;
using namespace std::chrono_literals;

// ---------------------------------------------------------------------------
// Fixture: a fresh in-memory exporter + SDK per test (mirrors the sibling
// integration suites).
// ---------------------------------------------------------------------------
class MoveItTraceTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    exporter_ = std::make_shared<robotops::InMemorySpanExporter>();
    robotops::Config config;
    config.service_name = "robotops_trace_moveit_test";
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

  static const robotops::SpanData * find_named(
    const std::vector<robotops::SpanData> & spans, const std::string & name)
  {
    for (const auto & s : spans) {
      if (s.name == name) {return &s;}
    }
    return nullptr;
  }

  static std::string str_attr(const robotops::SpanData & s, const std::string & key)
  {
    for (const auto & kv : s.attributes) {
      if (kv.first == key) {
        return kv.second.type() == robotops::AttributeValue::Type::String ?
               kv.second.string_value() : std::string{};
      }
    }
    return {};
  }

  static std::int64_t int_attr(const robotops::SpanData & s, const std::string & key)
  {
    for (const auto & kv : s.attributes) {
      if (kv.first == key) {
        return kv.second.type() == robotops::AttributeValue::Type::Int ?
               kv.second.int_value() : -1;
      }
    }
    return -1;
  }

  static rtm::TrajectoryInfo make_info()
  {
    rtm::TrajectoryInfo info;
    info.point_count = 12;            // robot.trajectory.point_count
    info.joint_count = 3;             // robot.joint.count
    info.joint_names = "shoulder_pan_joint,shoulder_lift_joint,elbow_joint";
    return info;
  }

  std::shared_ptr<robotops::InMemorySpanExporter> exporter_;
};

// ===========================================================================
// THE HEADLINE PROOF (ROB-426): the trace context survives MoveIt's queue ->
// execution-thread hop. A "move_action" span is opened and the context CAPTURED
// on thread A (the enqueue/push thread); the captured context is handed across
// to thread B (the TrajectoryExecutionManager execution thread) where it is
// RESTORED and the "moveit.execute" span opened, with a child
// "follow_joint_trajectory" span (the controller action client) opened beneath
// it. We assert the full chain nests end to end across the thread boundary:
//   move_action -> moveit.execute -> follow_joint_trajectory
// i.e. same trace_id throughout and the parent_span_id links are exactly right.
// ===========================================================================
TEST_F(MoveItTraceTest, ContextSurvivesQueueToExecutionThreadHop)
{
  rtm::TrajectoryExecutionTracer tracer;

  // A stable per-queued-trajectory key, exactly as the patch passes the
  // TrajectoryExecutionContext* address.
  int queued_trajectory{};
  const auto key = static_cast<rtm::TrajectoryExecutionTracer::Key>(&queued_trajectory);

  robotops::SpanContext move_action_ctx;

  // --- Thread A: the enqueue (push) thread, under the move_action span ---
  {
    robotops::SpanOptions opts;
    opts.kind = robotops::SpanKind::Server;   // the MoveGroup move_action server
    robotops::SpanGuard move_action("move_action", opts);
    move_action_ctx = move_action.span().context();
    ASSERT_TRUE(move_action_ctx.valid());

    // ENQUEUE: capture the active (move_action) context for this trajectory.
    tracer.on_enqueue(key);
    EXPECT_EQ(tracer.pending_count(), 1u) << "context captured on enqueue";

    // move_action closes here — BEFORE the execution thread runs. The capture is a
    // value snapshot, so the hop must still work even though the parent span's
    // RAII scope is already gone. (A strictly stronger guarantee than needing the
    // parent live across the boundary.)
  }

  // --- Thread B: the execution thread (executePart) ---
  robotops::SpanContext exec_ctx;
  robotops::SpanContext fjt_ctx;
  std::thread execution_thread(
    [&]()
    {
      // On a brand-new thread the SDK current-context is empty; restoring the
      // captured context is the only way the work below can nest under move_action.
      ASSERT_FALSE(robotops::current_context().valid());

      // RESTORE + open the execute span (bound to a local for the whole "execution").
      auto scope = tracer.on_execute(key, make_info());
      ASSERT_TRUE(scope.active());
      exec_ctx = scope.span().context();

      // Mimic what executePart() does next: sending the trajectory opens the
      // controller's FollowJointTrajectory action client. With the execute span
      // current, it must nest beneath it (no explicit parent threaded down).
      {
        robotops::SpanOptions copts;
        copts.kind = robotops::SpanKind::Client;
        robotops::SpanGuard fjt("follow_joint_trajectory", copts);
        fjt_ctx = fjt.span().context();
      }
    });
  execution_thread.join();

  EXPECT_EQ(tracer.pending_count(), 0u) << "capture consumed on execute";

  // ---- Assertions over the exported spans ----
  auto spans = flush_spans();
  const auto * move_action = find_named(spans, "move_action");
  const auto * execute = find_named(spans, "moveit.execute");
  const auto * fjt = find_named(spans, "follow_joint_trajectory");
  ASSERT_NE(move_action, nullptr);
  ASSERT_NE(execute, nullptr) << "the restored execute span must be exported";
  ASSERT_NE(fjt, nullptr);

  // (1) Everything is in ONE trace — the context crossed the thread/queue boundary.
  EXPECT_EQ(execute->context.trace_id, move_action->context.trace_id)
    << "execute span must share move_action's trace_id across the thread hop";
  EXPECT_EQ(fjt->context.trace_id, move_action->context.trace_id);

  // (2) The nesting links are exactly move_action -> moveit.execute -> fjt.
  EXPECT_EQ(execute->parent_span_id, move_action->context.span_id)
    << "moveit.execute must be a CHILD of move_action";
  EXPECT_EQ(fjt->parent_span_id, execute->context.span_id)
    << "the controller action client must be a CHILD of moveit.execute";

  // (3) The execute span is INTERNAL and carries the trajectory/joint semconv.
  EXPECT_EQ(execute->kind, robotops::SpanKind::Internal);
  EXPECT_EQ(int_attr(*execute, rtm::keys::kRobotTrajectoryPointCount), 12);
  EXPECT_EQ(int_attr(*execute, rtm::keys::kRobotJointCount), 3);
  EXPECT_EQ(
    str_attr(*execute, rtm::keys::kRobotJointName),
    "shoulder_pan_joint,shoulder_lift_joint,elbow_joint");
  EXPECT_EQ(
    str_attr(*execute, rtm::keys::kRobotComponentName), "trajectory_execution_manager");

  // Sanity: the captured ids really were threaded through (not coincidental names).
  EXPECT_EQ(exec_ctx.span_id, execute->context.span_id);
  EXPECT_EQ(fjt_ctx.span_id, fjt->context.span_id);
}

// ===========================================================================
// Multiple trajectories queued under DIFFERENT parents are restored to the
// RIGHT parent — the capture is keyed per queued trajectory, not global. This
// mirrors execute() running executePart(0), executePart(1) in sequence.
// ===========================================================================
TEST_F(MoveItTraceTest, EachQueuedTrajectoryRestoresItsOwnParent)
{
  rtm::TrajectoryExecutionTracer tracer;
  int t0{}, t1{};
  const auto k0 = static_cast<rtm::TrajectoryExecutionTracer::Key>(&t0);
  const auto k1 = static_cast<rtm::TrajectoryExecutionTracer::Key>(&t1);

  robotops::SpanContext p0_ctx, p1_ctx;
  {
    robotops::SpanGuard p0("move_action_A");
    p0_ctx = p0.span().context();
    tracer.on_enqueue(k0);
  }
  {
    robotops::SpanGuard p1("move_action_B");
    p1_ctx = p1.span().context();
    tracer.on_enqueue(k1);
  }
  EXPECT_EQ(tracer.pending_count(), 2u);

  // Execute both on a worker thread, in queue order, each its own scope.
  std::thread(
    [&]()
    {
      {
        auto s0 = tracer.on_execute(k0, make_info(), "moveit.execute.A");
      }
      {
        auto s1 = tracer.on_execute(k1, make_info(), "moveit.execute.B");
      }
    }).join();
  EXPECT_EQ(tracer.pending_count(), 0u);

  auto spans = flush_spans();
  const auto * eA = find_named(spans, "moveit.execute.A");
  const auto * eB = find_named(spans, "moveit.execute.B");
  ASSERT_NE(eA, nullptr);
  ASSERT_NE(eB, nullptr);
  EXPECT_EQ(eA->parent_span_id, p0_ctx.span_id) << "trajectory A -> parent A";
  EXPECT_EQ(eB->parent_span_id, p1_ctx.span_id) << "trajectory B -> parent B";
  EXPECT_NE(eA->context.trace_id, eB->context.trace_id) << "distinct parent traces";
}

// ===========================================================================
// discard() drops a captured-but-never-executed trajectory (the clear() path),
// so abandoned captures do not accumulate and a later execute does not adopt a
// stale parent.
// ===========================================================================
TEST_F(MoveItTraceTest, DiscardDropsCapturedContext)
{
  rtm::TrajectoryExecutionTracer tracer;
  int t{};
  const auto key = static_cast<rtm::TrajectoryExecutionTracer::Key>(&t);

  {
    robotops::SpanGuard p("move_action");
    tracer.on_enqueue(key);
  }
  EXPECT_EQ(tracer.pending_count(), 1u);

  tracer.discard(key);                       // clear() before execute()
  EXPECT_EQ(tracer.pending_count(), 0u);

  // A subsequent execute for the discarded key opens the execute span as a ROOT
  // (no captured parent), not under the stale move_action.
  std::thread(
    [&]()
    {
      auto scope = tracer.on_execute(key, make_info());
    }).join();

  auto spans = flush_spans();
  const auto * execute = find_named(spans, "moveit.execute");
  ASSERT_NE(execute, nullptr) << "execute span still emitted (useful as a root)";
  robotops::SpanContext zero{};
  EXPECT_EQ(execute->parent_span_id, zero.span_id) << "no parent after discard";
}

// ===========================================================================
// No active context at enqueue (push called outside any span) => nothing
// captured, and the execute span opens as a root. The integration never invents
// a parent.
// ===========================================================================
TEST_F(MoveItTraceTest, NoActiveContextMeansRootExecuteSpan)
{
  rtm::TrajectoryExecutionTracer tracer;
  int t{};
  const auto key = static_cast<rtm::TrajectoryExecutionTracer::Key>(&t);

  tracer.on_enqueue(key);                    // no surrounding span
  EXPECT_EQ(tracer.pending_count(), 0u) << "nothing to capture";

  std::thread(
    [&]()
    {
      auto scope = tracer.on_execute(key, make_info());
    }).join();

  auto spans = flush_spans();
  const auto * execute = find_named(spans, "moveit.execute");
  ASSERT_NE(execute, nullptr);
  robotops::SpanContext zero{};
  EXPECT_EQ(execute->parent_span_id, zero.span_id);
}

// ===========================================================================
// Zero-robot-impact: a DISABLED SDK produces no spans and never throws; the
// helper's bookkeeping stays inert (pending stays 0).
// ===========================================================================
TEST_F(MoveItTraceTest, DisabledSdkIsANoOp)
{
  robotops::shutdown();
  robotops::Config config;
  config.service_name = "disabled";
  config.enabled = false;                    // kill switch
  config.exporter = exporter_;
  robotops::init(std::move(config));

  rtm::TrajectoryExecutionTracer tracer;
  int t{};
  const auto key = static_cast<rtm::TrajectoryExecutionTracer::Key>(&t);

  // noexcept by contract — a disabled SDK simply degrades these to no-ops.
  {
    robotops::SpanGuard p("move_action");
    tracer.on_enqueue(key);
  }
  EXPECT_EQ(tracer.pending_count(), 0u) << "nothing captured when disabled";
  std::thread(
    [&]()
    {
      auto scope = tracer.on_execute(key, make_info());
      EXPECT_FALSE(scope.active());
    }).join();

  EXPECT_TRUE(flush_spans().empty()) << "a disabled SDK must emit no spans";
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
