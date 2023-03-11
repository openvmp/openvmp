cmake_minimum_required(VERSION 3.8)
project(openvmp_hardware_simulation_gazebo)

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
find_package(sdformat9 REQUIRED)
find_package(openvmp_hardware_configuration REQUIRED)
find_package(brake REQUIRED)
find_package(remote_actuator REQUIRED)
find_package(remote_encoder REQUIRED)
find_package(yaml-cpp REQUIRED)

link_directories(
  ${gazebo_dev_LIBRARY_DIRS}
  ${brake_LIBRARY_DIRS}
  ${remote_actuator_LIBRARY_DIRS}
  ${remote_encoder_LIBRARY_DIRS}
)

# Logic shared by all targets
include_directories(
  include
  ${brake_INCLUDE_DIRS}
  ${remote_actuator_INCLUDE_DIRS}
  ${remote_encoder_INCLUDE_DIRS}
  ${openvmp_hardware_configuration_INCLUDE_DIRS}
)

set(project_SOURCE_FILES
  src/plugin.cpp
  src/node.cpp
  src/joint.cpp
  src/actuator.cpp
  src/encoder.cpp
  src/brake.cpp
)

set(project_LIBRARIES
  ${rclcpp_LIBRARIES}
  ${brake_LIBRARIES}
  ${remote_actuator_LIBRARIES}
  ${remote_encoder_LIBRARIES}
)

set(project_DEPENDENCIES
  rclcpp
  gazebo_dev
  gazebo_ros
  sdformat9
  openvmp_hardware_configuration
  brake
  remote_actuator
  remote_encoder
)

# Library target
add_library(${PROJECT_NAME} SHARED ${project_SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ${project_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME} ${project_DEPENDENCIES})
ament_export_dependencies(${project_DEPENDENCIES})
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
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
