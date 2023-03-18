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

Device::Device(const std::string &joint, const std::string &driver_class,
               const std::string &id, const YAML::Node &node)
    : logger_{rclcpp::get_logger("openvmp_hardware_configuration::" + joint +
                                 "::" + id + ")")} {
  RCLCPP_DEBUG(logger_, "Initializing the device");

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

  if (node["driver"]) {
    driver_ = std::make_shared<Driver>(driver_class, id, node["driver"]);
  }
}

}  // namespace openvmp_hardware_configuration
