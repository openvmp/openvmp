/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-10-31
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_ROBOT_NODE_H
#define OPENVMP_ROBOT_NODE_H

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

namespace openvmp_robot {

class Node : public rclcpp::Node {
 public:
  Node(const std::string &name);
  virtual ~Node() {}

 protected:
  void preset_motion_walk();

 private:
};

}  // namespace openvmp_robot

#endif  // OPENVMP_ROBOT_NODE_H
