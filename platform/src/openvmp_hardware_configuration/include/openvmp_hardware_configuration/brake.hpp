/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-11-12
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_HARDWARE_CONFIGURATION_BRAKE_H
#define OPENVMP_HARDWARE_CONFIGURATION_BRAKE_H

#include "rclcpp/rclcpp.hpp"

namespace YAML {
class Node;
}

namespace openvmp_hardware_configuration {

class Brake {
 public:
  Brake(const std::string &, const YAML::Node &);

  virtual ~Brake() {}

  std::string get_path() const { return path_; }
  double get_torque() const { return torque_; }
  bool get_engaged_by_default() const { return engaged_by_default_; };

  // enum Type { SWITCH };

 private:
  rclcpp::Logger logger_;

  // Type type_;
  std::string path_;
  double torque_;
  bool engaged_by_default_;
};

}  // namespace openvmp_hardware_configuration

#endif  // OPENVMP_HARDWARE_CONFIGURATION_BRAKE_H