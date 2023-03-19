/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-16
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "openvmp_hardware_manager/node.hpp"

#include "openvmp_hardware_configuration/configuration.hpp"
#include "openvmp_hardware_manager/drivers.hpp"

namespace openvmp_hardware_manager {

Node::Node(std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> exec)
    : rclcpp::Node::Node("openvmp_hardware_manager"), exec_{exec} {
  this->declare_parameter("config_path", "config/hardware.yaml");
  this->get_parameter("config_path", param_config_path_);
  this->declare_parameter("use_fake_hardware", true);
  this->get_parameter("use_fake_hardware", param_use_fake_hardware_);

  auto configuration =
      std::make_shared<openvmp_hardware_configuration::Configuration>(
          param_config_path_.as_string());

  auto buses = configuration->get_buses();
  for (size_t i = 0; i < buses.size(); i++) {
    auto &bus_config = buses[i];

    RCLCPP_INFO(get_logger(), "bus: %s", bus_config->get_path().c_str());
    drivers_.emplace_back(this, exec, "bus", "driver_bus" + std::to_string(i),
                          "bus" + std::to_string(i), bus_config->get_path(),
                          bus_config->get_driver(),
                          param_use_fake_hardware_.as_bool());
  }

  auto joints = configuration->get_joints();
  for (auto &joint_config_it : joints) {
    auto &joint_name = joint_config_it.first;
    auto &joint = joint_config_it.second;
    RCLCPP_INFO(get_logger(), "joint: %s", joint_name.c_str());

    auto brake = joint->get_brake();
    if (brake) {
      drivers_.emplace_back(
          this, exec, "brake", "driver_" + joint_name + "_brake", joint_name,
          "", brake->get_driver(), param_use_fake_hardware_.as_bool());
    }

    auto actuator = joint->get_actuator();
    if (actuator) {
      drivers_.emplace_back(this, exec, "actuator",
                            "driver_" + joint_name + "_actuator", joint_name,
                            "", actuator->get_driver(),
                            param_use_fake_hardware_.as_bool());
    }

    auto encoder = joint->get_encoder();
    if (encoder) {
      drivers_.emplace_back(this, exec, "encoder",
                            "driver_" + joint_name + "_encoder", joint_name, "",
                            encoder->get_driver(),
                            param_use_fake_hardware_.as_bool());
    }
  }
}

}  // namespace openvmp_hardware_manager
