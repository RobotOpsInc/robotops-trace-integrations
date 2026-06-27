cmake_minimum_required(VERSION 3.8)
project(robotops_trace_semconv VERSION 0.2.0 LANGUAGES CXX)

# ROB-430 — robotics semantic conventions v0: header-only C++ key registry +
# Python mirror. Authoritative source of truth for the robot.* (portable concept)
# and ros.* (ROS mapping) attribute keys shared by every RobotOps integration.

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Header-only INTERFACE target so downstream packages can:
#   find_package(robotops_trace_semconv REQUIRED)
#   target_link_libraries(<tgt> robotops_trace_semconv::robotops_trace_semconv)
add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_17)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  INCLUDES DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

# Python mirror of the semantic-convention keys (also published to PyPI — see
# the pypi-publish workflow). Installs the robotops_trace_semconv python module.
ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # ament_copyright doesn't accept "Robot Ops Inc." as a holder (matches the
  # SDK-core + rclcpp integration convention), so skip just that linter.
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # Python mirror smoke test: importable + key strings exact.
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_conventions test/test_conventions.py)
endif()

ament_package()
