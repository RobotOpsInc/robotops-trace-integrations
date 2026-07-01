cmake_minimum_required(VERSION 3.8)
project(robotops_trace_rclcpp_autoattach VERSION 0.1.0 LANGUAGES CXX)

# ROB-453 — process-wide LD_PRELOAD auto-attach for stock rclcpp_action. Builds a
# preloadable shared object that interposes the non-templated ServerBase/ClientBase
# goal-UUID seams, so STOCK Nav2 emits CLIENT + SERVER action spans keyed by
# robot.action.goal_id and the agent joins the whole pipeline into one trace.

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# The interposer's *defined* base-class methods must byte-match the target's
# declarations, and it pins mangled ServerBase/ClientBase symbols — both are the
# jazzy (rclcpp_action v-jazzy) ABI. On humble the signatures differ (e.g.
# execute_goal_request_received), so DON'T build the DSO there — a humble variant
# is a follow-up. The package still "builds" (lint only) so colcon test passes.
if(DEFINED ENV{ROS_DISTRO} AND NOT "$ENV{ROS_DISTRO}" STREQUAL "humble")
  find_package(rclcpp_action REQUIRED)
  # The SDK core re-exports CURL::libcurl but its Config.cmake doesn't re-find it
  # (ROB-438) — same workaround as the other integration packages.
  find_package(CURL REQUIRED)
  find_package(robotops_trace_cpp REQUIRED)      # must ship DetachedSpan (ROB-443)
  find_package(robotops_trace_rclcpp REQUIRED)   # goal_id_to_string()
  find_package(robotops_trace_semconv REQUIRED)

  # A preloadable DSO — NOT linked by others, exports nothing. MUST keep default
  # ELF visibility so our ServerBase/ClientBase definitions interpose librclcpp_action.
  add_library(${PROJECT_NAME} SHARED src/rclcpp_action_autoattach.cpp)
  target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)
  ament_target_dependencies(${PROJECT_NAME}
    rclcpp_action
    robotops_trace_cpp
    robotops_trace_rclcpp
    robotops_trace_semconv
  )
  target_link_libraries(${PROJECT_NAME} ${CMAKE_DL_LIBS})  # dlsym / RTLD_NEXT

  install(
    TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
else()
  message(STATUS
    "robotops_trace_rclcpp_autoattach: skipping the DSO on ROS_DISTRO='$ENV{ROS_DISTRO}' "
    "(jazzy rclcpp_action ABI only; humble is a follow-up)")
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
