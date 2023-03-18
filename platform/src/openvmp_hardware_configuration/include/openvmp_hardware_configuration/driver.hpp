/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-17
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_HARDWARE_CONFIGURATION_DRIVER_H
#define OPENVMP_HARDWARE_CONFIGURATION_DRIVER_H

#include <map>
#include <memory>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"

namespace YAML {
class Node;
}

namespace openvmp_hardware_configuration {

typedef std::map<std::string, YAML::Node> driver_init_fields;

struct DriverInit {
  std::string service;
  std::string type;
  driver_init_fields fields;
};

class Driver {
 public:
  Driver(const std::string &driver_class, const std::string &id,
         const YAML::Node &);
  virtual ~Driver() {}

  const std::string &get_driver_class() const { return driver_class_; }
  const std::string &get_type() const { return type_; }
  const std::map<std::string, YAML::Node> &get_params() const {
    return params_;
  }
  const std::vector<DriverInit> &get_init() const { return init_; }

 private:
  rclcpp::Logger logger_;

  std::string driver_class_;
  std::string id_;

  std::string type_;
  std::map<std::string, YAML::Node> params_;
  std::vector<DriverInit> init_;
};

}  // namespace openvmp_hardware_configuration

#endif  // OPENVMP_HARDWARE_CONFIGURATION_DRIVER_H