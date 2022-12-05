/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-11-12
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "openvmp_hardware_configuration/encoder.hpp"

#include "yaml-cpp/yaml.h"

namespace openvmp_hardware_configuration {

Encoder::Encoder(const std::string &joint, const YAML::Node &node)
    : logger_{rclcpp::get_logger("openvmp_hardware_configuration::Encoder(" +
                                 joint + ")")} {
  if (!node || node.IsNull() || !node.IsMap()) {
    RCLCPP_ERROR(logger_, "Incorrect syntax");
    return;
  }

  // auto type = node["type"].as_string();
  // if (type == "absolute") {
  //   type_ = ABSOLUTE;
  // } else {
  //   RCLCPP_ERROR(logger_, "Invalid type");
  //   return;
  // }

  path_ = node["path"].as<std::string>();
  // ppr_ = node["ppr"].as<int>();
}

}  // namespace openvmp_hardware_configuration