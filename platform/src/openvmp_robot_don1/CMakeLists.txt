cmake_minimum_required(VERSION 3.10)
project(openvmp_robot_don1)

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
find_package(openvmp_robot REQUIRED)

include_directories(
  include
  ${openvmp_robot_INCLUDE_DIRS}
)

set(project_SOURCE_FILES
  src/main.cpp
  src/node.cpp
)

set(project_LIBRARIES
  ${rclcpp_LIBRARIES}
  ${openvmp_robot_LIBRARIES}
)

set(project_DEPENDENCIES
  rclcpp
  openvmp_robot
)


# Install the executable target
add_executable(${PROJECT_NAME} ${project_SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ${project_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME} ${project_DEPENDENCIES})
install(TARGETS
  ${PROJECT_NAME}
  DESTINATION bin/
)

# Install all other files
install(
  DIRECTORY config launch maps meshes models params worlds
  DESTINATION share/${PROJECT_NAME}
)
install(
  FILES model.config
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
