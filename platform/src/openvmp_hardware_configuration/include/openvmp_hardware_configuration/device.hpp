/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2023-02-25
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_HARDWARE_CONFIGURATION_DEVICE_H
#define OPENVMP_HARDWARE_CONFIGURATION_DEVICE_H

#include "rclcpp/rclcpp.hpp"

namespace YAML {
class Node;
}

namespace openvmp_hardware_configuration {

class Device {
 public:
  Device(const std::string &, const YAML::Node &, const std::string &id);
  virtual ~Device() {}

  std::string get_joint() const { return joint_; }
  std::string get_name() const { return name_; }
  std::string get_path() const { return path_; }

 protected:
  rclcpp::Logger logger_;

  // Type type_;
  std::string joint_;
  std::string name_;
  std::string path_;
};

}  // namespace openvmp_hardware_configuration

#endif  // OPENVMP_HARDWARE_CONFIGURATION_DEVICE_H
