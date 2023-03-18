/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-17
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "openvmp_hardware_configuration/bus.hpp"

#include "openvmp_hardware_configuration/driver.hpp"
#include "yaml-cpp/yaml.h"

namespace openvmp_hardware_configuration {

Bus::Bus(const YAML::Node &node, int index)
    : logger_{rclcpp::get_logger("openvmp_hardware_configuration::Bus")} {
  if (!node || node.IsNull() || !node.IsMap() || !node["path"]) {
    logger_ = rclcpp::get_logger("openvmp_motion_hardware::Bus");
    RCLCPP_ERROR(logger_, "Incorrect syntax of the bus");
    return;
  }

  path_ = node["path"].as<std::string>();
  logger_ =
      rclcpp::get_logger("openvmp_hardware_configuration::Bus(" + path_ + ")");

  if (node["driver"]) {
    driver_ = std::make_shared<Driver>("bus", "bus_" + std::to_string(index),
                                       node["driver"]);
  }
}

}  // namespace openvmp_hardware_configuration