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

Actuator::Actuator(const std::string &joint, const YAML::Node &node,
                   const std::string &id)
    : Device(joint, node, id) {
  auto type = node["type"].as<std::string>();
  if (type == "stepper") {
    type_ = STEPPER;
  } else if (type == "servo") {
    type_ = SERVO;
  } else {
    RCLCPP_ERROR(logger_, "Invalid type");
    return;
  }

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

std::string Actuator::get_prefix() const { return "/actuator/" + get_joint(); }

}  // namespace openvmp_hardware_configuration
