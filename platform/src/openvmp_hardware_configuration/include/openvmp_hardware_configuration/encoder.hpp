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

#include "openvmp_hardware_configuration/device.hpp"
#include "rclcpp/rclcpp.hpp"

namespace YAML {
class Node;
}

namespace openvmp_hardware_configuration {

class Encoder : public Device {
 public:
  Encoder(const std::string &, const YAML::Node &, const std::string &);
  virtual ~Encoder() {}

  std::string get_prefix() const;

  // enum Type { ABSOLUTE };

 private:
  // Type type_;
  // int ppr_;
};

}  // namespace openvmp_hardware_configuration

#endif  // OPENVMP_HARDWARE_CONFIGURATION_ENCODER_H
