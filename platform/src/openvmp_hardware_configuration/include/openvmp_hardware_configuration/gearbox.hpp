/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-11-12
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_HARDWARE_CONFIGURATION_GEARBOX_H
#define OPENVMP_HARDWARE_CONFIGURATION_GEARBOX_H

#include "rclcpp/rclcpp.hpp"

namespace YAML {
class Node;
}

namespace openvmp_hardware_configuration {

class Gearbox {
 public:
  Gearbox(const std::string &, const YAML::Node &);

  virtual ~Gearbox() {}

  enum Type { WORM, PLANETARY };
  static constexpr double WORM_SELF_LOCK_RATIO = 100.0;

  double get_ratio() const { return ratio_; }
  Type get_type() const { return type_; }

 private:
  rclcpp::Logger logger_;

  Type type_;
  double ratio_;
};

}  // namespace openvmp_hardware_configuration

#endif  // OPENVMP_HARDWARE_CONFIGURATION_GEARBOX_H