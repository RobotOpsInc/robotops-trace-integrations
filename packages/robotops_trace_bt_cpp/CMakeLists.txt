cmake_minimum_required(VERSION 3.8)
project(robotops_trace_bt_cpp VERSION 0.3.0 LANGUAGES CXX)

# ROB-424 — BehaviorTree.CPP integration: one RobotOps span per BT node
# EXECUTION, nested by tree structure, built on the robotops_trace_cpp SDK core.
# Fork-free: everything hangs off the PUBLIC BT::StatusChangeLogger observer seam
# (covers Nav2 + MoveIt Pro, both of which run upstream BehaviorTree.CPP).

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp REQUIRED)
# The SDK core links libcurl (for its OTLP exporter) and re-exports the
# CURL::libcurl IMPORTED target in its link interface. The published
# robotops_trace_cpp dev deb's Config.cmake does NOT re-find_package(CURL), so
# this find_package(CURL) is REQUIRED: without it the core's exported target
# references an undefined CURL::libcurl and the link fails. (Same workaround as
# robotops_trace_rclcpp; drop once the core's Config find_dependency(CURL) fix —
# ROB-438 — ships in a released deb.)
find_package(CURL REQUIRED)
# RobotOps SDK core (ros-<distro>-robotops-trace-cpp from the development apt
# channel) + shared semantic conventions (built from source in this workspace).
find_package(robotops_trace_cpp REQUIRED)
find_package(robotops_trace_semconv REQUIRED)

add_library(${PROJECT_NAME} src/robotops_trace_bt_cpp.cpp)
# Position-independent so this (static) lib can be linked into a SHARED object —
# e.g. the LD_PRELOAD interposer robotops_trace_bt_cpp_autoattach (ROB-451). Without
# it, aarch64 fails with "relocation R_AARCH64_ADR_PREL_PG_HI21 ... recompile with
# -fPIC" when the .a is pulled into a .so. (Same shared-consumability need as the
# core's ROB-439.)
set_target_properties(${PROJECT_NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

# All deps are PUBLIC: the public header exposes BT.CPP types (the TreeTracer
# base class + the BT::Tree ctor argument) and the SDK core / semconv keys, so
# consumers must transitively see them.
ament_target_dependencies(${PROJECT_NAME}
  behaviortree_cpp
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
ament_export_dependencies(behaviortree_cpp robotops_trace_cpp robotops_trace_semconv)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # ament_copyright doesn't accept "Robot Ops Inc." as a holder (matches the
  # SDK-core + rmw_robotops + rclcpp-integration convention), so skip just that
  # linter.
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_bt_cpp_integration test/test_bt_cpp_integration.cpp)
  if(TARGET test_bt_cpp_integration)
    target_link_libraries(test_bt_cpp_integration ${PROJECT_NAME})
    ament_target_dependencies(test_bt_cpp_integration
      behaviortree_cpp
      robotops_trace_cpp
    )
  endif()
endif()

ament_package()
