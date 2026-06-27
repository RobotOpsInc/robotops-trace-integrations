# robotops_trace_semconv

**Issue:** [ROB-430](https://linear.app/robotops/issue/ROB-430) &nbsp;•&nbsp; **Status:** 🟢 v0 dictionary

Robotics **semantic conventions** for RobotOps Trace: the attribute keys shared by
every integration package (rclcpp/rclpy/BT.CPP/ros2_control/MoveIt) and the SDK
cores. This package is the **authoritative source of truth** — the C++ header
(`include/robotops_trace_semconv/semconv.hpp`) and the mirrored Python module
(`robotops_trace_semconv/`) are kept in lockstep so attribute names cannot drift.
The human-readable design doc is
[`robotics-semantic-conventions-v0.md`](https://linear.app/robotops/issue/ROB-430)
(`tracehouse-mvp-planning/`).

This package ships **two ways**:

- **apt** (`ros-<distro>-robotops-trace-semconv`) via `ament_cmake` for ROS fleets
  (header-only `INTERFACE` target + the Python module).
- **PyPI** (`robotops-trace-semconv`) — the Python mirror, for pip-only users.

## Two namespaces

- **`robot.*`** — robotics-**general concept** keys. **Portable**: any robot has
  actions, transforms, joints, trajectories — not ROS-specific. This is the
  durable vocabulary TraceHouse/ROSQL display + filter on, and what a non-ROS
  framework maps onto later.
- **`ros.*`** — the **ROS mapping / implementation** keys (topic, gid, message
  type). Not portable; present only when the transport *is* ROS. A future
  framework gets its own `<framework>.*` mapping while reusing the same `robot.*`
  concepts.
- **Resource attributes** (`service.name`, `robot.id`) are set once per process on
  the OTel resource, not per span.

Types are OTel-compatible (`AttributeValue`): string, bool, int64, double, or
arrays thereof — no nested structs (poses decompose into `double[]`). Span
success/failure uses OTel `StatusCode`, **not** an attribute; `robot.action.result`
is the distinct *domain* outcome.

## Usage

C++ (header-only):

```cpp
#include <robotops_trace_semconv/semconv.hpp>
namespace sc = robotops::trace::semconv;
span.set_attribute(sc::kRobotActionGoalId, goal_uuid);
span.set_attribute(sc::kRobotActionResult, sc::action_result::kSucceeded);
```

Python:

```python
import robotops_trace_semconv as semconv
span.set_attribute(semconv.ROBOT_ACTION_GOAL_ID, goal_uuid)
span.set_attribute(semconv.ROBOT_ACTION_RESULT, semconv.ROBOT_ACTION_RESULT_SUCCEEDED)
```

## `robot.*` — concept keys (portable)

| Key | Type | C++ constant | Python constant |
|---|---|---|---|
| `robot.action.name` | string | `kRobotActionName` | `ROBOT_ACTION_NAME` |
| `robot.action.goal_id` | string | `kRobotActionGoalId` | `ROBOT_ACTION_GOAL_ID` |
| `robot.action.status` | string enum | `kRobotActionStatus` | `ROBOT_ACTION_STATUS` |
| `robot.action.result` | string enum | `kRobotActionResult` | `ROBOT_ACTION_RESULT` |
| `robot.callback.type` | string enum | `kRobotCallbackType` | `ROBOT_CALLBACK_TYPE` |
| `robot.transform.parent` | string | `kRobotTransformParent` | `ROBOT_TRANSFORM_PARENT` |
| `robot.transform.child` | string | `kRobotTransformChild` | `ROBOT_TRANSFORM_CHILD` |
| `robot.joint.name` | string[] | `kRobotJointName` | `ROBOT_JOINT_NAME` |
| `robot.joint.count` | int64 | `kRobotJointCount` | `ROBOT_JOINT_COUNT` |
| `robot.trajectory.point_count` | int64 | `kRobotTrajectoryPointCount` | `ROBOT_TRAJECTORY_POINT_COUNT` |
| `robot.trajectory.duration_ms` | double | `kRobotTrajectoryDurationMs` | `ROBOT_TRAJECTORY_DURATION_MS` |
| `robot.target.frame` | string | `kRobotTargetFrame` | `ROBOT_TARGET_FRAME` |
| `robot.target.position` | double[] | `kRobotTargetPosition` | `ROBOT_TARGET_POSITION` |
| `robot.target.orientation` | double[] | `kRobotTargetOrientation` | `ROBOT_TARGET_ORIENTATION` |
| `robot.object.id` | string | `kRobotObjectId` | `ROBOT_OBJECT_ID` |
| `robot.component.name` | string | `kRobotComponentName` | `ROBOT_COMPONENT_NAME` |

`robot.action.goal_id` is the **deterministic cross-process join key** (ROB-427):
RFC-4122 `8-4-4-4-12` lowercase, emitted identically on action client + server.

## `ros.*` — ROS mapping keys (implementation-specific)

| Key | Type | C++ constant | Python constant |
|---|---|---|---|
| `ros.node` | string | `kRosNode` | `ROS_NODE` |
| `ros.topic` | string | `kRosTopic` | `ROS_TOPIC` |
| `ros.service` | string | `kRosService` | `ROS_SERVICE` |
| `ros.message.type` | string | `kRosMessageType` | `ROS_MESSAGE_TYPE` |
| `ros.publisher_gid` | string | `kRosPublisherGid` | `ROS_PUBLISHER_GID` |
| `ros.source_timestamp` | int64 | `kRosSourceTimestamp` | `ROS_SOURCE_TIMESTAMP` |
| `ros.message.content_hash` | string | `kRosMessageContentHash` | `ROS_MESSAGE_CONTENT_HASH` |

`ros.publisher_gid` / `ros.source_timestamp` / `ros.message.content_hash` are the
best-effort **content-correlation keys** for topic/service hops (ROB-427); they
live under `ros.*` because they are properties of the ROS/DDS transport, not
robotics concepts.

## Resource attributes (set once per process, OTel resource)

| Key | Type | C++ constant | Python constant |
|---|---|---|---|
| `service.name` | string | `kServiceName` | `SERVICE_NAME` |
| `robot.id` | string | `kRobotId` | `ROBOT_ID` |

## Enumerated values

| Key | Values | C++ namespace | Python prefix |
|---|---|---|---|
| `robot.action.status` | `accepted` `executing` `succeeded` `aborted` `canceled` | `action_status::k*` | `ROBOT_ACTION_STATUS_*` |
| `robot.action.result` | `succeeded` `aborted` `canceled` | `action_result::k*` | `ROBOT_ACTION_RESULT_*` |
| `robot.callback.type` | `subscription` `timer` `service` `action` `client` | `callback_type::k*` | `ROBOT_CALLBACK_TYPE_*` |

## Versioning

`kSchemaVersion` / `SCHEMA_VERSION` tracks the package version. **v0 is
additive-only**: new keys may be added; existing key names and value enums are
stable. Breaking a key name/enum = a new major. ROSQL generalization (ROB-432)
keys its display/filter vocabulary off `robot.*` so it works for any robot, not
just ROS. The C++ header and Python module **MUST stay aligned**.
