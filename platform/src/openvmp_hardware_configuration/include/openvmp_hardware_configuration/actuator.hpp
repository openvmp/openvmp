/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-11-08
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_HARDWARE_CONFIGURATION_ACTUATOR_H
#define OPENVMP_HARDWARE_CONFIGURATION_ACTUATOR_H

#include "rclcpp/rclcpp.hpp"
#include "openvmp_hardware_configuration/device.hpp"

namespace YAML {
class Node;
}

namespace openvmp_hardware_configuration {

class Actuator : public Device {
 public:
  Actuator(const std::string &joint_name,
           const YAML::Node &node,
           const std::string &id);
  virtual ~Actuator() {}

  enum Type { STEPPER };

  std::string get_prefix() const;
  double get_torque() const { return torque_; }
  double get_torque_detent() const { return torque_detent_; }
  double get_torque_stalling() const { return torque_stalling_; }

 private:
  Type type_;
  double torque_;
  double torque_detent_;
  double torque_stalling_;
};

}  // namespace openvmp_hardware_configuration

#endif  // OPENVMP_HARDWARE_CONFIGURATION_ACTUATOR_H
