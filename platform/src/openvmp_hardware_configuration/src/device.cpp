/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-02-25
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "openvmp_hardware_configuration/device.hpp"

#include "yaml-cpp/yaml.h"

namespace openvmp_hardware_configuration {

Device::Device(const std::string &joint,
               const YAML::Node &node,
               const std::string &id)
    : logger_{rclcpp::get_logger("openvmp_hardware_configuration::" + id +")")} {
  if (!node || node.IsNull() || !node.IsMap()) {
    RCLCPP_ERROR(logger_, "Incorrect syntax");
    // TODO(clairbee): throw an exception?
    return;
  }

  joint_ = joint;

  name_ = id;
  if (node["name"]) {
    name_ = node["name"].as<std::string>();
  }

  path_ = "/" + name_;
  if (node["path"]) {
    path_ = node["path"].as<std::string>();
  }
}

}  // namespace openvmp_hardware_configuration
