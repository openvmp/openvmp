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

#include <memory>
#include <string>

#include "openvmp_hardware_configuration/driver.hpp"
#include "rclcpp/rclcpp.hpp"

namespace YAML {
class Node;
}

namespace openvmp_hardware_configuration {

class Device {
 public:
  Device(const std::string &joint, const std::string &driver_class,
         const std::string &id, const YAML::Node &config);
  virtual ~Device() {}

  std::string get_joint() const { return joint_; }
  std::string get_name() const { return name_; }
  std::string get_path() const { return path_; }

  std::shared_ptr<Driver> get_driver() { return driver_; }

 protected:
  rclcpp::Logger logger_;

  // Type type_;
  std::string joint_;
  std::string name_;
  std::string path_;

  std::shared_ptr<Driver> driver_;
};

}  // namespace openvmp_hardware_configuration

#endif  // OPENVMP_HARDWARE_CONFIGURATION_DEVICE_H
