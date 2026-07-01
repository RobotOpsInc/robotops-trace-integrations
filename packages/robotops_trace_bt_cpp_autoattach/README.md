# robotops_trace_bt_cpp_autoattach

**Issue:** [ROB-451](https://linear.app/robot-ops/issue/ROB-451) &nbsp;•&nbsp; **Status:** 🟡 first cut

Process-wide, **zero-code, fork-free** auto-attach of RobotOps BT tracing to
**stock** BehaviorTree.CPP apps — most importantly **Nav2's `bt_navigator`**,
which builds its `BT::Tree` internally where there is no external seam to attach
[`robotops_trace_bt_cpp`](../robotops_trace_bt_cpp)'s `TreeTracer`.

This is the "fully-automatic process-wide attach" the `robotops_trace_bt_cpp`
README defers to "the auto-init layer". You keep **stock upstream Nav2**.

```sh
export LD_PRELOAD=/opt/ros/$ROS_DISTRO/lib/librobotops_trace_bt_cpp_autoattach.so
export ROBOTOPS_TRACE_AUTOINIT=1              # SDK exports to the agent (or set endpoint)
export ROBOTOPS_OTLP_ENDPOINT=unix:///run/robotops/trace.sock
ros2 launch nav2_bringup tb4_loopback_simulation.launch.py
# bt_navigator now emits one span per BT node execution (ComputePathToPose,
# FollowPath, the recovery subtree, …) with bt.node_type / bt.status.
```

## Mechanism

`BT::BehaviorTreeFactory::createTree`, `createTreeFromText`, and
`createTreeFromFile` are **defined here** in a preloaded DSO (default ELF
visibility), so our symbols **interpose** `libbehaviortree_cpp.so`'s for every
caller. Each wrapper:

1. resolves the real implementation with `dlsym(RTLD_NEXT, "<mangled>")` — the
   exact BT.CPP-v4 mangled names — cast to a free-function ABI pointer (`this`
   first arg; the by-value `BT::Tree` return uses the Itanium sret convention,
   which the compiler matches for that signature);
2. calls it to build the tree;
3. constructs a `robotops::trace::bt::TreeTracer` on the result and stores it in a
   process-lifetime registry (`TreeTracer` is non-movable → heap-owned).

`createTreeFromText/File` funnel into `createTree` (and LTO may inline it), and
`libbehaviortree_cpp` is **not** built `-Bsymbolic`, so the internal call
re-enters us. A `thread_local` depth guard ensures **exactly one** `TreeTracer`
attaches — at the outermost frame, on the final (moved) tree. Attaching after the
move is safe: the tracer subscribes to `TreeNode` objects, which don't relocate
when `BT::Tree` moves.

## Zero robot impact

- Every attach is `try`/caught + the SDK ops are `noexcept` — a tracing fault
  **never** throws into BT.CPP / Nav2. The factory still builds the tree.
- Opt out with `ROBOTOPS_TRACE_BT_AUTOATTACH=0` (or the SDK-wide
  `ROBOTOPS_TRACE_ENABLED=0`): the wrappers still build trees, they just don't
  attach.
- `robotops::init()` is idempotent — safe whether or not the host also inits
  (or preloads `librobotops_trace_cpp_autoinit.so`).

## Limitations

- **BehaviorTree.CPP v4 only** (jazzy). The mangled symbols + the `TreeTracer`
  target v4; Nav2 on **humble** uses BT.CPP **v3** (different include root +
  mangled names) — a v3 build is a follow-up.
- **Symbol-pinned.** The three mangled names are pinned to BT.CPP's public ABI.
  A BT.CPP ABI change (rare within a major) needs a re-`nm`. Verified against
  `ros-jazzy-behaviortree-cpp 4.9.0`.
- **Preload order.** The DSO must be preloaded (so `RTLD_NEXT` finds the real
  symbols after it). `LD_PRELOAD` guarantees this.
- **Interpose, not upstream.** The clean long-term seam is an upstream
  observer-attach hook in Nav2's `BtActionServer` / BT.CPP
  ([ROB-452](https://linear.app/robot-ops/issue/ROB-452)); this interposer is the
  works-today path until that lands.

## Building & testing

ROS 2 is not on the host — build in Docker. Needs `behaviortree_cpp` (v4) + the
SDK core (`ros-<distro>-robotops-trace-cpp`, dev apt channel) +
`robotops_trace_bt_cpp` in one colcon workspace:

```sh
colcon build --packages-up-to robotops_trace_bt_cpp_autoattach
```

Validate end-to-end against the headless Nav2 loopback sim in `turtlebot_demo`:
`LD_PRELOAD` the DSO into the bringup, run a mission, and confirm the
`bt_navigator` BT-node spans appear in the trace.
