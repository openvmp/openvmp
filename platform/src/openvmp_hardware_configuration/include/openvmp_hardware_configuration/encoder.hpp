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

namespace YAML {
class Node;
}

namespace openvmp_hardware_configuration {

class Encoder {
 public:
  Encoder(const std::string &, const YAML::Node &);

  virtual ~Encoder() {}

  // enum Type { ABSOLUTE };

 private:
  rclcpp::Logger logger_;

  // Type type_;
  std::string path_;
  int ppr_;
};

}  // namespace openvmp_hardware_configuration

#endif  // OPENVMP_HARDWARE_CONFIGURATION_ENCODER_H