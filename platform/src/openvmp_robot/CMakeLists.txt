cmake_minimum_required(VERSION 3.10)
project(openvmp_robot)

# Compiler settings
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

include_directories(
  include
)

set(project_SOURCE_FILES
  src/node.cpp
  src/motion/preset_walk.cpp
)

set(project_LIBRARIES
  ${rclcpp_LIBRARIES}
)

set(project_DEPENDENCIES
  rclcpp
  rclcpp_action
  control_msgs
  trajectory_msgs
  builtin_interfaces
)

# Library target
add_library(${PROJECT_NAME} ${project_SOURCE_FILES})
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

install(
  # EXPORT ${PACKAGE_NAME}
  FILES
    scripts/package_sdf.py
    scripts/motion_demo_walk.py
  # DIRECTORY scripts
  DESTINATION lib/${PROJECT_NAME}
)

# Install all other files
install(
  DIRECTORY config launch worlds
  DESTINATION share/${PROJECT_NAME}
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
