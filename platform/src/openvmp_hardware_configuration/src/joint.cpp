/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-11-12
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "openvmp_hardware_configuration/joint.hpp"

#include "yaml-cpp/yaml.h"

namespace openvmp_hardware_configuration {

Joint::Joint(const YAML::Node &node)
    : logger_{rclcpp::get_logger("openvmp_hardware_configuration::Joint")} {
  if (!node || node.IsNull() || !node.IsMap() || !node["name"]) {
    logger_ = rclcpp::get_logger("openvmp_motion_hardware::Joint");
    RCLCPP_ERROR(logger_, "Incorrect syntax of the joint");
    return;
  }

  name_ = node["name"].as<std::string>();
  logger_ = rclcpp::get_logger("openvmp_hardware_configuration::Joint(" +
                               name_ + ")");

  actuator_ = std::make_shared<Actuator>(name_, node["actuator"], "joint_" + name_ + "_actuator");
  brake_ = std::make_shared<Brake>(name_, node["brake"], "joint_" + name_ + "_brake");
  encoder_ = std::make_shared<Encoder>(name_, node["encoder"], "joint_" + name_ + "_encoder");
  gearbox_ = std::make_shared<Gearbox>(name_, node["gearbox"], "joint_" + name_ + "_gearbox");
}

}  // namespace openvmp_hardware_configuration