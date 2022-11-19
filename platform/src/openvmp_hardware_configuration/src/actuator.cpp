/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-11-12
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "openvmp_hardware_configuration/actuator.hpp"

#include "yaml-cpp/yaml.h"

namespace openvmp_hardware_configuration {

Actuator::Actuator(const std::string &joint, const YAML::Node &node)
    : logger_{rclcpp::get_logger("openvmp_hardware_configuration::Actuator(" +
                                 joint + ")")} {
  if (!node || node.IsNull() || !node.IsMap()) {
    RCLCPP_ERROR(logger_, "Incorrect syntax");
    return;
  }

  auto type = node["type"].as<std::string>();
  if (type == "stepper") {
    type_ = STEPPER;
  } else {
    RCLCPP_ERROR(logger_, "Invalid type");
    return;
  }

  path_ = node["path"].as<std::string>();
  torque_ = node["torque"].as<double>();
  if (node["torqueDetent"]) {
    torque_detent_ = node["torqueDetent"].as<double>();
  } else {
    torque_detent_ = 0.0;
  }
  if (node["torqueStalling"]) {
    torque_stalling_ = node["torqueStalling"].as<double>();
  } else {
    torque_stalling_ = 0.0;
  }
}

}  // namespace openvmp_hardware_configuration