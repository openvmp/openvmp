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

Encoder::Encoder(const std::string &joint,
                 const YAML::Node &node,
                 const std::string &id)
    : Device(joint, node, id) {
  // auto type = node["type"].as_string();
  // if (type == "absolute") {
  //   type_ = ABSOLUTE;
  // } else {
  //   RCLCPP_ERROR(logger_, "Invalid type");
  //   return;
  // }

  // ppr_ = node["ppr"].as<int>();
}

}  // namespace openvmp_hardware_configuration
