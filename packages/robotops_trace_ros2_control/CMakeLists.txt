cmake_minimum_required(VERSION 3.8)
project(robotops_trace_ros2_control VERSION 0.2.0 LANGUAGES CXX)

# ROB-425 — ros2_control integration: RT-safe instrumentation of the controller /
# FollowJointTrajectory action-server boundary. One detached SERVER span per goal
# (accept -> result), carrying the canonical goal UUID (cross-process join key,
# ROB-427) + joint/trajectory semconv. Built on the robotops_trace_cpp SDK core.
# Ships a reusable helper (FollowJointTrajectoryTracer) + a carried patch (patches/)
# that wires it into stock joint_trajectory_controller. NOTHING runs in update().

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
# The SDK core links libcurl (for its OTLP exporter) and re-exports the
# CURL::libcurl IMPORTED target in its link interface. The published
# robotops_trace_cpp dev deb's Config.cmake does NOT re-find_package(CURL), so
# this find_package(CURL) is REQUIRED: without it the core's exported target
# references an undefined CURL::libcurl and the link fails. (Same workaround as
# robotops_trace_rclcpp / robotops_trace_bt_cpp; drop once the core's Config
# find_dependency(CURL) fix — ROB-438 — ships in a released deb.)
find_package(CURL REQUIRED)
# RobotOps SDK core (ros-<distro>-robotops-trace-cpp from the development apt
# channel; >=0.3.0 for the detached-span API) + shared semantic conventions
# (built from source in this workspace).
find_package(robotops_trace_cpp REQUIRED)
find_package(robotops_trace_semconv REQUIRED)

add_library(${PROJECT_NAME} src/robotops_trace_ros2_control.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
# This helper is designed to be linked INTO a controller plugin (a shared object
# loaded by pluginlib) — e.g. the carried joint_trajectory_controller patch. A
# static archive linked into a .so must be position-independent, so build the
# objects PIC even though the archive itself is static.
set_target_properties(${PROJECT_NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)

# All deps are PUBLIC: the public header exposes the FollowJointTrajectory goal
# type + the rclcpp_action GoalUUID type and the SDK core / semconv keys, so
# consumers (the patched controller, a custom controller) must transitively see them.
ament_target_dependencies(${PROJECT_NAME}
  control_msgs
  rclcpp_action
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
ament_export_dependencies(control_msgs rclcpp_action robotops_trace_cpp robotops_trace_semconv)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # ament_copyright doesn't accept "Robot Ops Inc." as a holder (matches the
  # SDK-core + rclcpp / bt_cpp integration convention), so skip just that linter.
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_ros2_control_integration test/test_ros2_control_integration.cpp)
  if(TARGET test_ros2_control_integration)
    target_link_libraries(test_ros2_control_integration ${PROJECT_NAME})
    ament_target_dependencies(test_ros2_control_integration
      control_msgs
      rclcpp_action
      robotops_trace_cpp
    )
  endif()
endif()

ament_package()
