/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-10-31
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "openvmp_robot/node.hpp"

namespace openvmp_robot {

Node::Node(const std::string &name)
    : rclcpp::Node::Node("openvmp_robot_" + name) {}

}  // namespace openvmp_robot
