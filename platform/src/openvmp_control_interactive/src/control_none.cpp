/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-12-10
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "openvmp_control_interactive/control_none.hpp"

namespace openvmp_control_interactive {

void NoneControl::init() {
  RCLCPP_INFO(node_->get_logger(), "NoneControl::init()");
}

void NoneControl::fini() {
  RCLCPP_INFO(node_->get_logger(), "NoneControl::fini()");
}

}  // namespace openvmp_control_interactive
