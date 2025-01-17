cmake_minimum_required(VERSION 3.8)
project(ur_painting_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME})
include_directories(include)

install(DIRECTORY
  launch
  config
  rviz
  worlds
  DESTINATION share/${PROJECT_NAME}/
)

add_executable(wall_coverage_node src/wall_coverage_node.cpp)
# add_executable(painting_node src/painting_node.cpp)

target_include_directories(wall_coverage_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# target_include_directories(painting_node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)

# target_compile_features(painting_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(
  wall_coverage_node
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs  
  nav_msgs
)

# ament_target_dependencies(
#   painting_node
#   rclcpp
#   moveit_core
#   moveit_ros_planning_interface
#   moveit_task_constructor_core
#   moveit_visual_tools
#   geometry_msgs  
#   tf2_geometry_msgs
#   nav_msgs
# )


# Create the node executable
add_executable(ur_motion_planner_node 
  src/ur_motion_planner_node.cpp
  src/ur_motion_planner.cpp
)

target_include_directories(ur_motion_planner_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(ur_motion_planner_node
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  moveit_task_constructor_core
  geometry_msgs
  tf2_geometry_msgs
)

# Install
install(TARGETS
  ur_motion_planner_node
  DESTINATION lib/${PROJECT_NAME}
)


install(TARGETS wall_coverage_node 
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  include/
  DESTINATION include/
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# install(TARGETS painting_node 
#   DESTINATION lib/${PROJECT_NAME})

#Install Python executables
install(PROGRAMS
ur_painting_planner/painting_node2.py
DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
