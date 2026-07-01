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

// ROB-451 — LD_PRELOAD auto-attach for BehaviorTree.CPP.
//
// Interposes the public BT::BehaviorTreeFactory::createTree* methods and attaches
// a robotops::trace::bt::TreeTracer to every tree the process builds — so STOCK
// Nav2 `bt_navigator` (and any BT.CPP app) emits BT-node spans with ZERO code
// changes and no fork. This is the process-wide attach the robotops_trace_bt_cpp
// README defers to "the auto-init layer".
//
//   LD_PRELOAD=/opt/ros/<distro>/lib/librobotops_trace_bt_cpp_autoattach.so
//   ROBOTOPS_TRACE_AUTOINIT=1 ros2 launch nav2_bringup ... (see README)
//
// HOW IT WORKS
//   * We *define* the three factory methods in this preloaded DSO (default ELF
//     visibility), so our symbols override libbehaviortree_cpp.so's for every
//     caller. Inside, we fetch the real implementation via dlsym(RTLD_NEXT, ...)
//     — the exact mangled names for BT.CPP v4 — cast to a free-function ABI
//     pointer (Itanium: `this` is the first arg, the by-value BT::Tree return
//     uses the hidden sret pointer), call it, then attach a TreeTracer.
//   * createTreeFromText/File funnel into createTree (and LTO may inline it), and
//     libbehaviortree_cpp is NOT built -Bsymbolic, so the internal call re-enters
//     us. A thread-local depth guard ensures exactly ONE TreeTracer attaches, at
//     the OUTERMOST frame — on the final (moved) tree. Attaching after the move is
//     safe: TreeTracer subscribes to TreeNode objects, which do not relocate when
//     BT::Tree is moved.
//
// ZERO ROBOT IMPACT: every attach is try/caught + noexcept; a tracing failure can
// never throw into BT.CPP / Nav2. Opt out with ROBOTOPS_TRACE_BT_AUTOATTACH=0 (or
// the SDK-wide ROBOTOPS_TRACE_ENABLED=0) — the factory methods still build trees,
// they just don't attach.
//
// DISTRO: the mangled symbols + TreeTracer target BehaviorTree.CPP v4 (jazzy).
// Humble Nav2 uses BT.CPP v3 (a different include root + mangled names) — a v3
// build is a separate follow-up.

#include <behaviortree_cpp/bt_factory.h>

#include <robotops_trace_bt_cpp/robotops_trace_bt_cpp.hpp>
#include <robotops_trace/trace.hpp>

#include <dlfcn.h>

#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace
{

bool env_off(const char * name)
{
  const char * v = std::getenv(name);
  if (v == nullptr || *v == '\0') {
    return false;
  }
  return std::strcmp(v, "0") == 0 || std::strcmp(v, "false") == 0 ||
         std::strcmp(v, "off") == 0 || std::strcmp(v, "FALSE") == 0;
}

bool autoattach_enabled()
{
  // Read once. Opt out via our own switch or the SDK-wide kill switch.
  static const bool on =
    !env_off("ROBOTOPS_TRACE_BT_AUTOATTACH") && !env_off("ROBOTOPS_TRACE_ENABLED");
  return on;
}

// The SDK core init() is idempotent + noexcept; call it once so a preloaded-only
// deployment (no explicit init) still exports. If the host also calls init(), the
// second call is a logged no-op.
void ensure_init()
{
  static std::once_flag once;
  std::call_once(once, [] {robotops::init();});
}

// TreeTracer is non-copyable AND non-movable → heap-own it for the process
// lifetime. We rarely need to remove one (a tree lives as long as its navigator).
std::mutex g_mu;
std::vector<std::unique_ptr<robotops::trace::bt::TreeTracer>> g_tracers;

// createTreeFromText/File re-enter createTree; attach only at the outermost frame.
thread_local int g_depth = 0;

struct DepthGuard
{
  DepthGuard() {++g_depth;}
  ~DepthGuard() {--g_depth;}
  bool outermost() const {return g_depth == 1;}
};

void attach_tracer(const BT::Tree & tree) noexcept
{
  if (!autoattach_enabled()) {
    return;
  }
  try {
    if (tree.rootNode() == nullptr) {
      return;  // nothing was built (e.g. the real fn failed)
    }
    ensure_init();
    auto tracer = std::make_unique<robotops::trace::bt::TreeTracer>(tree);
    std::lock_guard<std::mutex> lock(g_mu);
    g_tracers.push_back(std::move(tracer));
  } catch (...) {
    // Zero robot impact: a tracing fault must never perturb tree construction.
  }
}

// Free-function ABI signatures of the (non-virtual) member functions: `this` is
// the first argument; the by-value BT::Tree return is handled by the compiler's
// sret convention for this exact signature, matching the member ABI.
using RealCreate = BT::Tree (*)(BT::BehaviorTreeFactory *, const std::string &, BT::Blackboard::Ptr);
using RealFromText =
  BT::Tree (*)(BT::BehaviorTreeFactory *, const std::string &, BT::Blackboard::Ptr);
using RealFromFile =
  BT::Tree (*)(BT::BehaviorTreeFactory *, const std::filesystem::path &, BT::Blackboard::Ptr);

// Exact mangled names from `nm -D libbehaviortree_cpp.so` (BT.CPP 4.9.0). NOTE
// the blackboard is passed BY VALUE (std::shared_ptr<Blackboard>, not const&).
constexpr const char * kCreate =
  "_ZN2BT19BehaviorTreeFactory10createTreeERKNSt7__cxx1112basic_"
  "stringIcSt11char_traitsIcESaIcEEESt10shared_ptrINS_10BlackboardEE";
constexpr const char * kFromText =
  "_ZN2BT19BehaviorTreeFactory18createTreeFromTextERKNSt7__cxx1112basic_"
  "stringIcSt11char_traitsIcESaIcEEESt10shared_ptrINS_10BlackboardEE";
constexpr const char * kFromFile =
  "_ZN2BT19BehaviorTreeFactory18createTreeFromFileERKNSt10filesystem7__cxx114"
  "pathESt10shared_ptrINS_10BlackboardEE";

}  // namespace

// ---------------------------------------------------------------------------
// Interposed definitions. Default visibility → these override
// libbehaviortree_cpp.so for every caller in the process.
// ---------------------------------------------------------------------------
namespace BT
{

Tree BehaviorTreeFactory::createTree(const std::string & tree_name, Blackboard::Ptr blackboard)
{
  DepthGuard guard;
  static const auto real = reinterpret_cast<RealCreate>(::dlsym(RTLD_NEXT, kCreate));
  Tree tree = real ? real(this, tree_name, std::move(blackboard)) : Tree{};
  if (guard.outermost()) {
    attach_tracer(tree);
  }
  return tree;
}

Tree BehaviorTreeFactory::createTreeFromText(const std::string & text, Blackboard::Ptr blackboard)
{
  DepthGuard guard;
  static const auto real = reinterpret_cast<RealFromText>(::dlsym(RTLD_NEXT, kFromText));
  Tree tree = real ? real(this, text, std::move(blackboard)) : Tree{};
  if (guard.outermost()) {
    attach_tracer(tree);
  }
  return tree;
}

Tree BehaviorTreeFactory::createTreeFromFile(
  const std::filesystem::path & file_path, Blackboard::Ptr blackboard)
{
  DepthGuard guard;
  static const auto real = reinterpret_cast<RealFromFile>(::dlsym(RTLD_NEXT, kFromFile));
  Tree tree = real ? real(this, file_path, std::move(blackboard)) : Tree{};
  if (guard.outermost()) {
    attach_tracer(tree);
  }
  return tree;
}

}  // namespace BT
