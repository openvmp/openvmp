/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-17
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "openvmp_hardware_configuration/driver.hpp"

#include "yaml-cpp/yaml.h"

namespace openvmp_hardware_configuration {

Driver::Driver(const std::string &driver_class, const std::string &id,
               const YAML::Node &node)
    : logger_{rclcpp::get_logger("openvmp_hardware_configuration::" + id +
                                 ")")},
      driver_class_{driver_class},
      id_{id} {
  RCLCPP_DEBUG(logger_, "Initializing the driver");

  if (!node || node.IsNull() || !node.IsMap()) {
    RCLCPP_ERROR(logger_, "Incorrect syntax");
    // TODO(clairbee): throw an exception?
    return;
  }

  if (node["type"]) {
    type_ = node["type"].as<std::string>();
  } else {
    type_ = "fake";
  }
  RCLCPP_DEBUG(logger_, "The driver type is %s", type_.c_str());

  if (node["init"]) {
    RCLCPP_DEBUG(logger_, "Reading the driver initialization sequence");
    for (auto init_param : node["init"]) {
      RCLCPP_DEBUG(logger_, "Found a driver initialization step");

      const auto &service = init_param["service"].as<std::string>();
      const auto &type = init_param["type"].as<std::string>();
      RCLCPP_DEBUG(logger_, "Found a driver initialization step: %s: %s",
                   service.c_str(), type.c_str());
      DriverInit init = {.service = service, .type = type, .fields = {}};

      for (const auto &field : init_param) {
        const auto &key = field.first.as<std::string>();
        if (key == "service" || key == "type") {
          continue;
        }
        RCLCPP_DEBUG(logger_, "Found a driver initialization step field: %s",
                     key.c_str());
        init.fields.insert({key, field.second});
        RCLCPP_INFO(logger_, "init: %s: %s", key.c_str(),
                    field.second.as<std::string>().c_str());
      }
      init_.push_back(init);
    }
  }

  for (auto param_it : node) {
    auto key = param_it.first.as<std::string>();
    if (key == "type" || key == "init") continue;
    auto value = param_it.second.as<std::string>();

    RCLCPP_INFO(logger_, "param: %s: %s", key.c_str(), value.c_str());
    params_.insert({key, param_it.second});
  }
  RCLCPP_DEBUG(logger_, "Done initializing the driver");
}

}  // namespace openvmp_hardware_configuration
