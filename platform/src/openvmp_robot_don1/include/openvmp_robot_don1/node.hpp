/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-10-31
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_ROBOT_DON1_NODE_H
#define OPENVMP_ROBOT_DON1_NODE_H

#include <memory>
#include <string>

#include "openvmp_robot/node.hpp"
#include "rclcpp/rclcpp.hpp"

namespace openvmp_robot_don1 {

class Node : public openvmp_robot::Node {
 public:
  Node();

 private:
};

}  // namespace openvmp_robot_don1

#endif  // OPENVMP_ROBOT_DON1_NODE_H
