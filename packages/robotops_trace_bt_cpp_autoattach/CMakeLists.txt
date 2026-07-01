cmake_minimum_required(VERSION 3.8)
project(robotops_trace_bt_cpp_autoattach VERSION 0.1.0 LANGUAGES CXX)

# ROB-451 — process-wide LD_PRELOAD auto-attach for BehaviorTree.CPP. Builds a
# single preloadable shared object that interposes BT::BehaviorTreeFactory::
# createTree* and drops a robotops_trace_bt_cpp TreeTracer on every tree — so
# STOCK Nav2 bt_navigator emits BT-node spans with no code change and no fork.

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp REQUIRED)
# The SDK core re-exports CURL::libcurl in its link interface but its published
# Config.cmake does not re-find_package(CURL) (ROB-438). Find it so the exported
# target resolves — same workaround as robotops_trace_bt_cpp/rclcpp.
find_package(CURL REQUIRED)
find_package(robotops_trace_cpp REQUIRED)
find_package(robotops_trace_bt_cpp REQUIRED)

# A preloadable DSO — NOT a library others link against, so it exports nothing.
# It MUST keep default ELF visibility (do not add -fvisibility=hidden): our
# BT::BehaviorTreeFactory::createTree* definitions must be visible to interpose
# libbehaviortree_cpp.so.
add_library(${PROJECT_NAME} SHARED src/bt_autoattach.cpp)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)
ament_target_dependencies(${PROJECT_NAME}
  behaviortree_cpp
  robotops_trace_cpp
  robotops_trace_bt_cpp
)
target_link_libraries(${PROJECT_NAME} ${CMAKE_DL_LIBS})  # dlsym / RTLD_NEXT

# Install to lib/ so it lands at /opt/ros/<distro>/lib/lib${PROJECT_NAME}.so —
# the path run.sh / the launch env LD_PRELOADs.
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # ament_copyright doesn't accept "Robot Ops Inc." as a holder (matches the rest
  # of the monorepo) — skip just that linter.
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
