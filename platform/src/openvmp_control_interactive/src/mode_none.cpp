/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-12-10
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "openvmp_control_interactive/mode_none.hpp"

namespace openvmp_control_interactive {

void NoneMode::enter(std::shared_ptr<ControlImpl> from) {
  ControlImpl::enter(from);

  RCLCPP_DEBUG(node_->get_logger(), "NoneMode::enter()");
}

void NoneMode::leave(std::shared_ptr<ControlImpl> to) {
  RCLCPP_DEBUG(node_->get_logger(), "NoneMode::leave()");

  ControlImpl::leave(to);
}

}  // namespace openvmp_control_interactive
