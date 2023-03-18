/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-16
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_HARDWARE_MANAGER_NODE_H
#define OPENVMP_HARDWARE_MANAGER_NODE_H

#include <memory>
#include <string>

#include "openvmp_hardware_manager/driver.hpp"
#include "rclcpp/rclcpp.hpp"

namespace openvmp_hardware_manager {

class Node : public rclcpp::Node {
 public:
  Node(std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> exec);

 private:
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> exec_;

  // node parameters
  rclcpp::Parameter param_config_path_;
  rclcpp::Parameter param_use_fake_hardware_;

  // state
  std::vector<Driver> drivers_;
};

}  // namespace openvmp_hardware_manager

#endif  // OPENVMP_HARDWARE_MANAGER_NODE_H
