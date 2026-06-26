cmake_minimum_required(VERSION 3.8)
project(robotops_trace_rclcpp VERSION 0.1.0 LANGUAGES CXX)

# ROB-422 — rclcpp integration: executor/callback spans + the rclcpp_action
# goal-UUID correlation key, built on the robotops_trace_cpp SDK core. Fork-free:
# everything here wraps PUBLIC rclcpp / rclcpp_action seams.

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
# The SDK core links libcurl (for its OTLP exporter) and re-exports the
# CURL::libcurl IMPORTED target in its link interface, but its Config.cmake does
# NOT re-find_package(CURL). So consumers must pull CURL in themselves or the
# core's exported target fails to resolve. (TODO: upstream a find_dependency(CURL)
# into robotops_trace_cpp's Config so this is unnecessary once the core ships.)
find_package(CURL REQUIRED)
# RobotOps SDK core (built from source in the combined colcon workspace until
# ros-<distro>-robotops-trace-cpp is published to apt) + shared semantic
# conventions.
find_package(robotops_trace_cpp REQUIRED)
find_package(robotops_trace_semconv REQUIRED)

add_library(${PROJECT_NAME} src/robotops_trace_rclcpp.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

# All deps are PUBLIC: the integration is mostly header-only templates, so
# consumers must transitively see rclcpp / rclcpp_action / the SDK core / semconv.
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
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
ament_export_dependencies(rclcpp rclcpp_action robotops_trace_cpp robotops_trace_semconv)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # ament_copyright doesn't accept "Robot Ops Inc." as a holder (matches the
  # SDK-core + rmw_robotops convention), so skip just that linter.
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(example_interfaces REQUIRED)
  find_package(std_msgs REQUIRED)

  ament_add_gtest(test_rclcpp_integration test/test_rclcpp_integration.cpp)
  if(TARGET test_rclcpp_integration)
    target_link_libraries(test_rclcpp_integration ${PROJECT_NAME})
    ament_target_dependencies(test_rclcpp_integration
      rclcpp
      rclcpp_action
      robotops_trace_cpp
      example_interfaces
      std_msgs
    )
  endif()
endif()

ament_package()
