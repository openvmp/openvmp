/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-11-12
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_HARDWARE_CONFIGURATION_JOINT_H
#define OPENVMP_HARDWARE_CONFIGURATION_JOINT_H

#include "openvmp_hardware_configuration/actuator.hpp"
#include "openvmp_hardware_configuration/brake.hpp"
#include "openvmp_hardware_configuration/encoder.hpp"
#include "openvmp_hardware_configuration/gearbox.hpp"
#include "rclcpp/rclcpp.hpp"

namespace YAML {
class Node;
}

namespace openvmp_hardware_configuration {

class Joint {
 public:
  Joint(const YAML::Node &);

  virtual ~Joint() {}

  const std::string &get_name() const { return name_; }

  std::shared_ptr<Actuator> get_actuator() { return actuator_; }
  std::shared_ptr<Brake> get_brake() { return brake_; }
  std::shared_ptr<Gearbox> get_gearbox() { return gearbox_; }
  // double get_brake_torque() const;

 private:
  rclcpp::Logger logger_;

  std::string name_;
  std::shared_ptr<Actuator> actuator_;
  std::shared_ptr<Brake> brake_;
  std::shared_ptr<Encoder> encoder_;
  std::shared_ptr<Gearbox> gearbox_;
};

}  // namespace openvmp_hardware_configuration

#endif  // OPENVMP_HARDWARE_CONFIGURATION_JOINT_H