/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-17
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_HARDWARE_CONFIGURATION_BUS_H
#define OPENVMP_HARDWARE_CONFIGURATION_BUS_H

#include "openvmp_hardware_configuration/driver.hpp"
#include "rclcpp/rclcpp.hpp"

namespace YAML {
class Node;
}

namespace openvmp_hardware_configuration {

class Bus {
 public:
  Bus(const YAML::Node &, int index);
  virtual ~Bus() {}

  const std::string &get_path() const { return path_; }

  std::shared_ptr<Driver> get_driver() { return driver_; }

 private:
  rclcpp::Logger logger_;

  std::string path_;
  std::shared_ptr<Driver> driver_;
};

}  // namespace openvmp_hardware_configuration

#endif  // OPENVMP_HARDWARE_CONFIGURATION_BUS_H