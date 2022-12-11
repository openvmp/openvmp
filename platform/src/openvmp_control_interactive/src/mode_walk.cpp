/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-12-10
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "openvmp_control_interactive/mode_walk.hpp"

namespace openvmp_control_interactive {

void WalkMode::enter(std::shared_ptr<ControlImpl> from) {
  (void)from;

  ControlImpl::enter(from);

  RCLCPP_INFO(node_->get_logger(), "WalkMode::enter()");
}

void WalkMode::leave(std::shared_ptr<ControlImpl> to) {
  (void)to;

  RCLCPP_INFO(node_->get_logger(), "WalkMode::leave()");

  ControlImpl::leave(to);
}

void WalkMode::processFeedback_(
    const std::string &marker_name,
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr
        &feedback) {
  (void)marker_name;
  (void)feedback;
}

}  // namespace openvmp_control_interactive
