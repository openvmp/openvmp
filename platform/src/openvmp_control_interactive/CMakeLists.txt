cmake_minimum_required(VERSION 3.10)
project(openvmp_control_interactive)

# Compiler settings
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Logic shared by all targets
include_directories(
  include
)

set(project_SOURCE_FILES
  src/main.cpp
  src/node.cpp
  src/link.cpp
  src/menu.cpp
  src/mode_none.cpp
  src/mode_full.cpp
  src/mode_walk.cpp
  src/mode_drive.cpp
  src/mode_hug.cpp
  src/mode_hang.cpp
  src/mode_grab.cpp
  src/mode_lift.cpp
  src/mode_camera.cpp
  src/control.cpp
  src/control_none.cpp
  src/control_traj_vel.cpp
  src/control_position.cpp
)

set(project_DEPENDENCIES
  rclcpp
  std_msgs
  control_msgs
  geometry_msgs
  visualization_msgs
  interactive_markers
  tf2
  tf2_geometry_msgs
  trajectory_msgs
  builtin_interfaces
)

# Executable target
add_executable(${PROJECT_NAME} src/main.cpp ${project_SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ${rclcpp_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME} ${project_DEPENDENCIES})
install(TARGETS
  ${PROJECT_NAME}
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
