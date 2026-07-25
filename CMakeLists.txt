cmake_minimum_required(VERSION 3.8)
project(avatar_challenge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

set(dependencies
  rclcpp
  geometry_msgs
  visualization_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_msgs
  tf2
  tf2_eigen
  tf2_ros
  tf2_geometry_msgs
)

# Geometry, YAML parsing and redundancy resolution live in a library so they can
# be unit tested without a running move_group.
add_library(shape_tracing SHARED
  src/path_sampler.cpp
  src/yaml_loader.cpp
  src/redundancy.cpp
  src/markers.cpp
)
target_include_directories(shape_tracing PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(shape_tracing yaml-cpp)
ament_target_dependencies(shape_tracing ${dependencies} Eigen3)

add_executable(shape_tracer src/shape_tracer_node.cpp)
target_link_libraries(shape_tracer shape_tracing)
ament_target_dependencies(shape_tracer ${dependencies} Eigen3)

install(TARGETS shape_tracing
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS shape_tracer
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

# Launch-time helper shared by start.launch.py and trace_shapes.launch.py.
ament_python_install_package(${PROJECT_NAME}_launch)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_shape_geometry test/test_shape_geometry.cpp)
  target_link_libraries(test_shape_geometry shape_tracing)
  ament_target_dependencies(test_shape_geometry ${dependencies} Eigen3)
endif()

ament_export_include_directories(include)
ament_export_libraries(shape_tracing)
ament_export_dependencies(${dependencies} Eigen3)
ament_package()
