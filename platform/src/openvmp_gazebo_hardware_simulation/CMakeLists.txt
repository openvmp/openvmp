cmake_minimum_required(VERSION 3.8)
project(openvmp_gazebo_hardware_simulation)

# Compiler settings
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(switch_interface REQUIRED)
find_package(sdformat9 REQUIRED)

link_directories(${gazebo_dev_LIBRARY_DIRS})

# Logic shared by all targets
include_directories(
  include
)

set(project_SOURCE_FILES
  src/brakes.cpp
  src/plugin.cpp
)

set(project_LIBRARIES
  ${rclcpp_LIBRARIES}
)

set(project_DEPENDENCIES
  rclcpp
  gazebo_dev
  gazebo_ros
  switch_interface
  sdformat9
)

# Library target
add_library(${PROJECT_NAME} SHARED ${project_SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ${project_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME} ${project_DEPENDENCIES})
ament_export_targets(${PROJECT_NAME}_library HAS_LIBRARY_TARGET)
ament_export_dependencies(${project_DEPENDENCIES})
ament_export_include_directories(include)
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}_library
  LIBRARY DESTINATION lib
)
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
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
