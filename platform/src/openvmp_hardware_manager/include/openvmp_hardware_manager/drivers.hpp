/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-16
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_HARDWARE_MANAGER_DRIVERS_H
#define OPENVMP_HARDWARE_MANAGER_DRIVERS_H

#include <any>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "openvmp_hardware_configuration/driver.hpp"
#include "rclcpp/rclcpp.hpp"

namespace openvmp_hardware_manager {

struct DriverInfo {
  std::function<std::shared_ptr<void>(rclcpp::Node *)> factory;
  std::vector<std::pair<std::string, YAML::Node>> params;
  std::vector<openvmp_hardware_configuration::DriverInit> init;
};

typedef std::map<std::string, DriverInfo> Drivers;

extern std::map<std::string, Drivers> driver_classes;

}  // namespace openvmp_hardware_manager

#endif /* OPENVMP_HARDWARE_MANAGER_DRIVERS_H */
