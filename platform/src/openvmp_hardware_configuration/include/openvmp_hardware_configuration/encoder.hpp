/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-11-12
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_HARDWARE_CONFIGURATION_ENCODER_H
#define OPENVMP_HARDWARE_CONFIGURATION_ENCODER_H

#include "rclcpp/rclcpp.hpp"
#include "openvmp_hardware_configuration/device.hpp"

namespace YAML {
class Node;
}

namespace openvmp_hardware_configuration {

class Encoder : public Device {
 public:
  Encoder(const std::string &, const YAML::Node &, const std::string &);
  virtual ~Encoder() {}

  // enum Type { ABSOLUTE };

 private:
  // Type type_;
  // int ppr_;
};

}  // namespace openvmp_hardware_configuration

#endif  // OPENVMP_HARDWARE_CONFIGURATION_ENCODER_H
