/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-17
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_HARDWARE_MANAGER_DRIVER_H
#define OPENVMP_HARDWARE_MANAGER_DRIVER_H

#include <string>

#include "modbus/srv/configured_holding_register_write.hpp"
#include "openvmp_hardware_configuration/configuration.hpp"
#include "rclcpp/rclcpp.hpp"

namespace openvmp_hardware_manager {

class Driver {
 public:
  Driver(rclcpp::Node *parent,
         std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> exec,
         const std::string &driver_class, const std::string &node_name,
         const std::string &id,
         std::shared_ptr<openvmp_hardware_configuration::Driver> config,
         bool use_fake_hardware);
  virtual ~Driver() = default;

 private:
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> exec_;
  std::shared_ptr<rclcpp::Node> node_;

  rclcpp::Client<modbus::srv::ConfiguredHoldingRegisterWrite>::SharedPtr
      clnt_modbus_chrw_;

  std::shared_ptr<void> instance_;
};

}  // namespace openvmp_hardware_manager

#endif /* OPENVMP_HARDWARE_MANAGER_DRIVER_H */
