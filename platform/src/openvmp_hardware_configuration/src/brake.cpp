/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-11-12
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "openvmp_hardware_configuration/brake.hpp"

#include "yaml-cpp/yaml.h"

namespace openvmp_hardware_configuration {

Brake::Brake(const std::string &joint, const YAML::Node &node)
    : logger_{rclcpp::get_logger("openvmp_hardware_configuration::Brake(" +
                                 joint + ")")} {
  if (!node || node.IsNull() || !node.IsMap()) {
    RCLCPP_ERROR(logger_, "Incorrect syntax");
    return;
  }

  // auto type = node["type"].as<std::string>();

  // if (type == "switch") {
  //   type_ = SWITCH;
  // } else {
  //   RCLCPP_ERROR(logger_, "Invalid type");
  //   return;
  // }

  path_ = node["path"].as<std::string>();
  torque_ = node["torque"].as<double>();
  if (node["engagedByDefault"]) {
    engaged_by_default_ = node["engagedByDefault"].as<bool>();
  }
}

}  // namespace openvmp_hardware_configuration