/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-11-12
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "openvmp_hardware_configuration/gearbox.hpp"

#include "yaml-cpp/yaml.h"

namespace openvmp_hardware_configuration {

Gearbox::Gearbox(const std::string &joint, const YAML::Node &node,
                 const std::string &id)
    : Device(joint, "gearbox", id, node) {
  RCLCPP_DEBUG(logger_, "Initializing the gearbox");
  auto type = node["type"].as<std::string>();
  if (type == "worm") {
    type_ = WORM;
  } else if (type == "planetary") {
    type_ = PLANETARY;
  } else {
    RCLCPP_ERROR(logger_, "Invalid type");
    type_ = INVALID;
    ratio_ = 0.0;
    return;
  }

  ratio_ = node["ratio"].as<double>();
}

}  // namespace openvmp_hardware_configuration