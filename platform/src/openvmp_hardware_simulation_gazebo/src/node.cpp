/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-11-12
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "openvmp_hardware_simulation_gazebo/node.hpp"

#include "rclcpp/node_options.hpp"

namespace openvmp_hardware_simulation_gazebo {

Node::Node(const std::string &ns)
    : rclcpp::Node::Node("openvmp_hardware_simulation_gazebo", ns, rclcpp::NodeOptions().use_intra_process_comms(true)) {}

}  // namespace openvmp_hardware_simulation_gazebo
