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
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "rclcpp_action/types.hpp"

#include "robotops_trace/trace.hpp"
#include "robotops_trace_ros2_control/robotops_trace_ros2_control.hpp"

namespace rtc = robotops::trace::ros2_control;
namespace ar = robotops::trace::semconv::action_result;
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using namespace std::chrono_literals;

// ---------------------------------------------------------------------------
// Helpers to build a real, synthetic FollowJointTrajectory goal.
// ---------------------------------------------------------------------------

// A fixed 16-byte goal UUID whose canonical RFC-4122 rendering is known, so the
// test pins the EXACT string the agent (ROB-427) joins on.
constexpr std::array<std::uint8_t, 16> kUuidBytes{
  0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef,
  0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef};
constexpr const char * kUuidCanonical = "01234567-89ab-cdef-0123-456789abcdef";

// The exported span name for an "/arm_controller/follow_joint_trajectory" action.
constexpr const char * kArmSpanName =
  "/arm_controller/follow_joint_trajectory follow_joint_trajectory";

rclcpp_action::GoalUUID make_uuid()
{
  rclcpp_action::GoalUUID uuid;
  for (std::size_t i = 0; i < uuid.size(); ++i) {
    uuid[i] = kUuidBytes[i];
  }
  return uuid;
}

// A 3-joint, 4-point trajectory goal — the joint/trajectory semconv source.
FollowJointTrajectory::Goal make_goal()
{
  FollowJointTrajectory::Goal goal;
  goal.trajectory.joint_names = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint"};
  goal.trajectory.points.resize(4);
  return goal;
}

// ---------------------------------------------------------------------------
// Fixture: a fresh in-memory exporter + SDK per test (mirrors the bt_cpp suite).
// ---------------------------------------------------------------------------
class Ros2ControlTraceTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    exporter_ = std::make_shared<robotops::InMemorySpanExporter>();
    robotops::Config config;
    config.service_name = "robotops_trace_ros2_control_test";
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

  std::shared_ptr<robotops::InMemorySpanExporter> exporter_;
};

// ===========================================================================
// The headline proof: a goal driven accept -> succeed yields exactly ONE
// SERVER span carrying the canonical goal_id + the joint/trajectory semconv
// attrs + result=succeeded (Ok status), nested under an EXPLICIT parent.
// ===========================================================================
TEST_F(Ros2ControlTraceTest, AcceptThenSucceedEmitsOneStitchableServerSpan)
{
  rtc::FollowJointTrajectoryTracer tracer;
  const auto uuid = make_uuid();
  const auto goal = make_goal();
  const std::string action_name = "/arm_controller/follow_joint_trajectory";

  // An explicit parent span the goal must nest under (proves explicit parentage;
  // in production the server span is normally a root joined cross-process by
  // goal_id, but the helper supports an explicit in-process parent).
  robotops::DetachedSpan parent = robotops::start_detached_span("controller_scope");
  const robotops::SpanContext parent_ctx = parent.context();

  // --- accept boundary (non-RT) ---
  tracer.on_goal_accepted(uuid, goal, action_name, &parent_ctx);
  EXPECT_EQ(tracer.active_count(), 1u) << "span should be held open from accept";
  // Still open => not yet exported (one span per goal EXECUTION, not per poll).
  EXPECT_EQ(find_named(flush_spans(), action_name + " follow_joint_trajectory"), nullptr);

  // --- result boundary (non-RT): goal succeeded ---
  tracer.on_result(uuid, ar::kSucceeded);
  EXPECT_EQ(tracer.active_count(), 0u) << "span should be closed on result";

  parent.end();

  auto spans = flush_spans();
  const auto * goal_span = find_named(spans, action_name + " follow_joint_trajectory");
  const auto * parent_span = find_named(spans, "controller_scope");
  ASSERT_NE(goal_span, nullptr) << "exactly one goal span must be exported";
  ASSERT_NE(parent_span, nullptr);

  // It is the action SERVER side.
  EXPECT_EQ(goal_span->kind, robotops::SpanKind::Server);

  // Canonical goal UUID — the deterministic cross-process join key (ROB-427).
  EXPECT_EQ(str_attr(*goal_span, rtc::keys::kRobotActionGoalId), kUuidCanonical);
  EXPECT_EQ(str_attr(*goal_span, rtc::keys::kRobotActionName), action_name);

  // Joint + trajectory semantic conventions, sourced from the goal trajectory.
  EXPECT_EQ(
    str_attr(*goal_span, rtc::keys::kRobotJointName),
    "shoulder_pan_joint,shoulder_lift_joint,elbow_joint");
  EXPECT_EQ(int_attr(*goal_span, rtc::keys::kRobotJointCount), 3);
  EXPECT_EQ(int_attr(*goal_span, rtc::keys::kRobotTrajectoryPointCount), 4);

  // Terminal outcome + span status.
  EXPECT_EQ(str_attr(*goal_span, rtc::keys::kRobotActionResult), std::string(ar::kSucceeded));
  EXPECT_EQ(goal_span->status_code, robotops::StatusCode::Ok);

  // Nested under the EXPLICIT parent (same trace, parent's span id).
  EXPECT_EQ(goal_span->context.trace_id, parent_span->context.trace_id);
  EXPECT_EQ(goal_span->parent_span_id, parent_span->context.span_id);
}

// ===========================================================================
// An aborted goal -> result=aborted and an Error span status.
// ===========================================================================
TEST_F(Ros2ControlTraceTest, AbortedGoalIsAnErrorSpan)
{
  rtc::FollowJointTrajectoryTracer tracer;
  const auto uuid = make_uuid();

  tracer.on_goal_accepted(uuid, make_goal(), "/arm_controller/follow_joint_trajectory");
  tracer.on_result(uuid, ar::kAborted);

  auto spans = flush_spans();
  const auto * s = find_named(spans, kArmSpanName);
  ASSERT_NE(s, nullptr);
  EXPECT_EQ(str_attr(*s, rtc::keys::kRobotActionResult), std::string(ar::kAborted));
  EXPECT_EQ(s->status_code, robotops::StatusCode::Error);
}

// ===========================================================================
// A canceled goal -> result=canceled and an Unset span status (a cancel is
// neither a success nor a server error).
// ===========================================================================
TEST_F(Ros2ControlTraceTest, CanceledGoalLeavesStatusUnset)
{
  rtc::FollowJointTrajectoryTracer tracer;
  const auto uuid = make_uuid();

  tracer.on_goal_accepted(uuid, make_goal(), "/arm_controller/follow_joint_trajectory");
  tracer.on_result(uuid, ar::kCanceled);

  auto spans = flush_spans();
  const auto * s = find_named(spans, kArmSpanName);
  ASSERT_NE(s, nullptr);
  EXPECT_EQ(str_attr(*s, rtc::keys::kRobotActionResult), std::string(ar::kCanceled));
  EXPECT_EQ(s->status_code, robotops::StatusCode::Unset);
}

// ===========================================================================
// on_result is idempotent: the action monitor timer may fire repeatedly after
// the terminal; only one span is ever emitted and extra calls are no-ops.
// ===========================================================================
TEST_F(Ros2ControlTraceTest, ResultIsIdempotent)
{
  rtc::FollowJointTrajectoryTracer tracer;
  const auto uuid = make_uuid();

  tracer.on_goal_accepted(uuid, make_goal(), "/c/fjt");
  tracer.on_result(uuid, ar::kSucceeded);
  tracer.on_result(uuid, ar::kSucceeded);   // second fire — must be a no-op
  tracer.on_result(uuid, ar::kAborted);     // and must not flip the outcome

  auto spans = flush_spans();
  std::size_t n = 0;
  for (const auto & s : spans) {
    if (s.name == "/c/fjt follow_joint_trajectory") {++n;}
  }
  EXPECT_EQ(n, 1u) << "exactly one span regardless of repeated on_result()";
  EXPECT_EQ(tracer.active_count(), 0u);
}

// ===========================================================================
// Zero-robot-impact: a DISABLED SDK must produce no spans and never throw, and
// the helper's bookkeeping must not perturb the controller (active_count stays 0).
// ===========================================================================
TEST_F(Ros2ControlTraceTest, DisabledSdkIsANoOp)
{
  robotops::shutdown();
  robotops::Config config;
  config.service_name = "disabled";
  config.enabled = false;            // kill switch
  config.exporter = exporter_;
  robotops::init(std::move(config));

  rtc::FollowJointTrajectoryTracer tracer;
  const auto uuid = make_uuid();

  // The helper methods are noexcept, so a disabled SDK simply degrades them to
  // no-ops: no spans, no perturbation. (No EXPECT_NO_THROW wrapper — the noexcept
  // contract already forbids throwing.)
  tracer.on_goal_accepted(uuid, make_goal(), "/c/fjt");
  tracer.on_result(uuid, ar::kSucceeded);
  EXPECT_TRUE(flush_spans().empty()) << "a disabled SDK must emit no spans";
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
