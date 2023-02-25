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

Gearbox::Gearbox(const std::string &joint,
                 const YAML::Node &node,
                 const std::string &id)
    : Device(joint, node, id) {
  auto type = node["type"].as<std::string>();
  if (type == "worm") {
    type_ = WORM;
  } else if (type == "planetary") {
    type_ = PLANETARY;
  } else {
    RCLCPP_ERROR(logger_, "Invalid type");
    return;
  }

  ratio_ = node["ratio"].as<double>();
}

}  // namespace openvmp_hardware_configuration