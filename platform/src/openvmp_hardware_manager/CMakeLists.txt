cmake_minimum_required(VERSION 3.10)
project(openvmp_hardware_manager)

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
find_package(yaml-cpp REQUIRED)
find_package(openvmp_hardware_configuration REQUIRED)
find_package(remote_modbus_rtu REQUIRED)
find_package(em2rs_driver REQUIRED)
find_package(r413d08_driver REQUIRED)
find_package(remote_serial REQUIRED)
find_package(remote_serial_bus REQUIRED)
find_package(amt21_driver REQUIRED)
find_package(remote_actuator REQUIRED)
find_package(remote_encoder REQUIRED)
find_package(remote_brake REQUIRED)
find_package(remote_switch REQUIRED)
find_package(remote_brake_switch REQUIRED)
find_package(remote_microcontroller REQUIRED)
find_package(remote_actuator_pwm REQUIRED)
find_package(openvmp_hardware_configuration REQUIRED)


# Logic shared by all targets
include_directories(
  include
  ${remote_modbus_INCLUDE_DIRS}
  ${remote_modbus_rtu_INCLUDE_DIRS}
  ${em2rs_driver_INCLUDE_DIRS}
  ${em2rs_driver_native_INCLUDE_DIRS}
  ${remote_serial_INCLUDE_DIRS}
  ${remote_serial_bus_INCLUDE_DIRS}
  ${amt21_driver_INCLUDE_DIRS}
  ${amt21_driver_native_INCLUDE_DIRS}
  ${remote_actuator_INCLUDE_DIRS}
  ${remote_encoder_INCLUDE_DIRS}
  ${remote_brake_INCLUDE_DIRS}
  ${remote_switch_INCLUDE_DIRS}
  ${remote_brake_switch_INCLUDE_DIRS}
  ${remote_microcontroller_INCLUDE_DIRS}
  ${remote_actuator_pwm_INCLUDE_DIRS}
  ${openvmp_hardware_configuration_INCLUDE_DIRS}
)

set(project_SOURCE_FILES
  src/main.cpp
  src/node.cpp
  src/drivers.cpp
  src/driver.cpp
)

set(project_LIBRARIES
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${remote_modbus_LIBRARIES}
  ${remote_modbus_LIBRARIES}
  ${remote_modbus_rtu_LIBRARIES}
  ${remote_modbus_rtu_native_LIBRARIES}
  ${remote_serial_LIBRARIES}
  ${remote_serial_native_LIBRARIES}
  ${remote_serial_bus_LIBRARIES}
  ${remote_serial_bus_native_LIBRARIES}
  ${remote_actuator_LIBRARIES}
  ${remote_actuator_fake_LIBRARIES}
  ${remote_encoder_LIBRARIES}
  ${remote_encoder_fake_LIBRARIES}
  ${em2rs_driver_LIBRARIES}
  ${em2rs_driver_native_LIBRARIES}
  ${remote_brake_fake_LIBRARIES}
  ${remote_brake_switch_LIBRARIES}
  ${remote_brake_switch_native_LIBRARIES}
  ${openvmp_hardware_configuration_LIBRARIES}
  ${amt21_driver_LIBRARIES}
  ${remote_microcontroller_LIBRARIES}
  ${remote_actuator_pwm_LIBRARIES}
)

set(project_DEPENDENCIES
  rclcpp
  std_msgs
  remote_modbus
  remote_modbus_rtu
  em2rs_driver
  remote_serial
  remote_serial_bus
  amt21_driver
  remote_encoder
  remote_actuator
  remote_brake
  remote_switch
  remote_brake_switch
  remote_microcontroller
  remote_actuator_pwm
  openvmp_hardware_configuration
)


# Executable target
add_executable(${PROJECT_NAME} ${project_SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ${project_LIBRARIES})
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
