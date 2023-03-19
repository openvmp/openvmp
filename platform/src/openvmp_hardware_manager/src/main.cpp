/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-16
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "openvmp_hardware_manager/node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  std::shared_ptr<openvmp_hardware_manager::Node> node;

  auto thread = std::thread([exec, &node]() {
    node = std::make_shared<openvmp_hardware_manager::Node>(exec);
    exec->add_node(node);
  });

  exec->spin();
  exec.reset();

  rclcpp::shutdown();
  return 0;
}
