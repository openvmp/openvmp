/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-09-24
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_HARDWARE_SIMULATION_GAZEBO_NODE_H
#define OPENVMP_HARDWARE_SIMULATION_GAZEBO_NODE_H

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

namespace openvmp_hardware_simulation_gazebo {

class Node : public rclcpp::Node {
 public:
  Node(const std::string &ns);

 private:
};

}  // namespace openvmp_hardware_simulation_gazebo

#endif  // OPENVMP_HARDWARE_SIMULATION_GAZEBO_NODE_H
