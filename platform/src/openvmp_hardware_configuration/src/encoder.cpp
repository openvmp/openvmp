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

Encoder::Encoder(const std::string &joint, const YAML::Node &node,
                 const std::string &id)
    : Device(joint, "encoder", id, node) {
  RCLCPP_DEBUG(logger_, "Initializing the encoder");
  // auto type = node["type"].as_string();
  // if (type == "absolute") {
  //   type_ = ABSOLUTE;
  // } else {
  //   RCLCPP_ERROR(logger_, "Invalid type");
  //   return;
  // }

  // ppr_ = node["ppr"].as<int>();
}

std::string Encoder::get_prefix() const { return "/encoder/" + get_joint(); }

}  // namespace openvmp_hardware_configuration
