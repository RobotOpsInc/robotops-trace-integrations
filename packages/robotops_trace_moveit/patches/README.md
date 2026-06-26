# MoveIt async patch (★)

**Issue:** [ROB-426](https://linear.app/robotops/issue/ROB-426) — placeholder.

This directory will hold the not-yet-upstream patch against MoveIt's
`TrajectoryExecutionManager` that lets RobotOps Trace capture/restore the trace
context across MoveIt's internal async boundary (the one hop stock
instrumentation can't see).

Per spec §3.4 this is the **sole** carried-fork exception: the patch is
minimized and pushed upstream over time. CI applies it against stock MoveIt per
distro to prove it still applies cleanly.

No patch file yet — landed in ROB-426. Expected layout once it exists:

```
patches/
  0001-trajectory-execution-manager-context-capture.patch
  apply.sh        # applies the patch against the stock MoveIt source tree
```
