/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-11-12
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_HARDWARE_CONFIGURATION_H
#define OPENVMP_HARDWARE_CONFIGURATION_H

#include <map>
#include <string>

#include "openvmp_hardware_configuration/bus.hpp"
#include "openvmp_hardware_configuration/joint.hpp"
#include "rclcpp/rclcpp.hpp"

namespace openvmp_hardware_configuration {

class Configuration {
 public:
  Configuration(const std::string &config_filename);

  virtual ~Configuration() {}

  std::map<std::string, std::shared_ptr<Joint>> &get_joints() {
    return joints_;
  }

  std::vector<std::shared_ptr<Bus>> &get_buses() { return buses_; }

 private:
  rclcpp::Logger logger_;

  std::map<std::string, std::shared_ptr<Joint>> joints_;
  std::vector<std::shared_ptr<Bus>> buses_;
};

}  // namespace openvmp_hardware_configuration

#endif  // OPENVMP_HARDWARE_CONFIGURATION_H