cmake_minimum_required(VERSION 3.8)
project(robotops_trace_rclcpp VERSION 0.1.0 LANGUAGES CXX)

# ROB-422 STUB — builds/installs a placeholder library so the package is a valid
# colcon/ament/bloom target. The real hooks (and the find_package() calls for
# robotops_trace_cpp + robotops_trace_semconv + the target framework) land in
# ROB-422; they are intentionally NOT wired here so the scaffold does not depend
# on the not-yet-published SDK core at configure time.

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)

# TODO(ROB-422): find_package(robotops_trace_cpp REQUIRED)
# TODO(ROB-422): find_package(robotops_trace_semconv REQUIRED)
# TODO(ROB-422): find_package(<target framework> REQUIRED)

add_library(${PROJECT_NAME} src/placeholder.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

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

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
