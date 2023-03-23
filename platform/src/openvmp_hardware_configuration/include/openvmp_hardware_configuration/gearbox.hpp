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

#include "openvmp_hardware_configuration/device.hpp"
#include "rclcpp/rclcpp.hpp"

namespace YAML {
class Node;
}

namespace openvmp_hardware_configuration {

class Gearbox : public Device {
 public:
  Gearbox(const std::string &, const YAML::Node &, const std::string &);
  virtual ~Gearbox() {}

  enum Type { INVALID, WORM, PLANETARY };
  static constexpr double WORM_SELF_LOCK_RATIO = 100.0;

  double get_ratio() const { return ratio_; }
  Type get_type() const { return type_; }

 private:
  Type type_;
  double ratio_;
};

}  // namespace openvmp_hardware_configuration

#endif  // OPENVMP_HARDWARE_CONFIGURATION_GEARBOX_H