/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-11-12
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "openvmp_hardware_configuration/configuration.hpp"

#include "yaml-cpp/yaml.h"

namespace openvmp_hardware_configuration {

Configuration::Configuration(const std::string &config_filename)
    : logger_{rclcpp::get_logger("openvmp_hardware_configuration")} {
  YAML::Node config = YAML::LoadFile(config_filename);
  if (!config || config.IsNull() || !config.IsMap()) {
    RCLCPP_ERROR(logger_, "Failed to parse: %s", config_filename.c_str());
    return;
  }

  auto joints = config["joints"];
  for (auto joint : joints) {
    RCLCPP_DEBUG(logger_, "Found a joint");

    if (!joint["name"]) {
      RCLCPP_ERROR(logger_, "Each joint must have a name");
      continue;
    }

    auto name = joint["name"].as<std::string>();
    joints_.insert({name, std::make_shared<Joint>(joint)});
  }
}

}  // namespace openvmp_hardware_configuration