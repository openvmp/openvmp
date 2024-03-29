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

Brake::Brake(const std::string &joint, const YAML::Node &node,
             const std::string &id)
    : Device(joint, "brake", id, node) {
  RCLCPP_DEBUG(logger_, "Initializing the brake");
  // auto type = node["type"].as<std::string>();

  // if (type == "switch") {
  //   type_ = SWITCH;
  // } else {
  //   RCLCPP_ERROR(logger_, "Invalid type");
  //   return;
  // }

  torque_ = node["torque"].as<double>();
  if (node["engagedByDefault"]) {
    engaged_by_default_ = node["engagedByDefault"].as<bool>();
  } else {
    engaged_by_default_ = false;
  }
}

std::string Brake::get_prefix() const { return "/brake/" + get_joint(); }

}  // namespace openvmp_hardware_configuration
