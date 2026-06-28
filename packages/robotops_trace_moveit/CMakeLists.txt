cmake_minimum_required(VERSION 3.8)
project(robotops_trace_moveit VERSION 0.2.0 LANGUAGES CXX)

# ROB-426 — MoveIt integration: capture/restore of the trace context across
# MoveIt's TrajectoryExecutionManager (TEM) internal async boundary (the ★ one
# residual no-public-hook case). The active context is captured when a trajectory
# is enqueued (push, caller thread) and restored when it executes (executePart,
# execution thread), so move_action -> moveit.execute -> FollowJointTrajectory
# nests end to end across the queue. Built on the robotops_trace_cpp SDK core.
#
# Ships a reusable, MoveIt-AGNOSTIC helper (TrajectoryExecutionTracer) + a carried
# patch (patches/) wiring it into stock moveit_ros_planning's TEM. The helper
# itself links ONLY the SDK core + semconv (it deals in trace contexts + a POD),
# so it builds and is unit-tested WITHOUT a MoveIt install; MoveIt is needed only
# to build the patched TEM (a separate, optional proof — see patches/README.md).

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
# The SDK core links libcurl (for its OTLP exporter) and re-exports the
# CURL::libcurl IMPORTED target in its link interface. The published
# robotops_trace_cpp dev deb's Config.cmake does NOT re-find_package(CURL), so
# this find_package(CURL) is REQUIRED: without it the core's exported target
# references an undefined CURL::libcurl and the link fails. (Same workaround as
# the rclcpp / bt_cpp / ros2_control integrations; drop once the core's Config
# find_dependency(CURL) fix — ROB-438 — ships in a released deb.)
find_package(CURL REQUIRED)
# RobotOps SDK core (ros-<distro>-robotops-trace-cpp from the development apt
# channel; >=0.3.0 for the capture_context/ScopedContext async-context API) +
# shared semantic conventions (built from source in this workspace).
find_package(robotops_trace_cpp REQUIRED)
find_package(robotops_trace_semconv REQUIRED)
#
# NOTE: moveit_ros_planning / moveit_core are NOT find_package'd here. This helper
# is MoveIt-agnostic by design (it includes no MoveIt header), so it builds with
# only the trace deps present. MoveIt enters the picture only via the carried TEM
# patch (patches/), which is built separately against an apt MoveIt underlay. The
# package.xml still declares the MoveIt deps because that is the framework the
# patch targets (mirrors the ros2_control integration, which depends on the
# controller framework in package.xml but does not find_package it for the helper).

add_library(${PROJECT_NAME} src/robotops_trace_moveit.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
# The carried patch links this helper INTO moveit_trajectory_execution_manager, a
# MoveIt SHARED library. A static archive linked into a .so must be
# position-independent, so build the objects PIC even though the archive is static.
set_target_properties(${PROJECT_NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)

# Both deps are PUBLIC: the public header exposes SDK-core context/span types
# (Context, ScopedContext, SpanGuard, Span) and the semconv keys, so consumers
# (the patched TEM, a custom integrator) must transitively see them.
ament_target_dependencies(${PROJECT_NAME}
  robotops_trace_cpp
  robotops_trace_semconv
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(robotops_trace_cpp robotops_trace_semconv)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # ament_copyright doesn't accept "Robot Ops Inc." as a holder (matches the
  # SDK-core + sibling integrations' convention), so skip just that linter.
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_moveit_integration test/test_moveit_integration.cpp)
  if(TARGET test_moveit_integration)
    target_link_libraries(test_moveit_integration ${PROJECT_NAME})
    ament_target_dependencies(test_moveit_integration robotops_trace_cpp)
  endif()
endif()

ament_package()
