/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-12-10
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "openvmp_control_interactive/control_traj_vel.hpp"

#include <algorithm>

namespace openvmp_control_interactive {

void TrajVelControl::init() {
  RCLCPP_INFO(node_->get_logger(), "TrajVelControl::init()");

  trajectory_commands_ =
      node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(
          node_->get_effective_namespace() +
              "/trajectory_controller/joint_trajectory",
          10);
  if (!trajectory_commands_) {
    RCLCPP_ERROR(
        node_->get_logger(),
        "Failed to subscrive to /trajectory_controller/joint_trajectory");
  }
  // position_commands_ = create_publisher<std_msgs::msg::Float64MultiArray>(
  //     this->get_effective_namespace() + "/position_controller/commands", 10);
  // velocity_commands_ = create_publisher<std_msgs::msg::Float64MultiArray>(
  //     this->get_effective_namespace() + "/velocity_controller/commands", 10);
}

void TrajVelControl::fini() {
  RCLCPP_INFO(node_->get_logger(), "TrajVelControl::fini()");

  // TODO(clairbee): make sure this is a proper cleanup
  trajectory_commands_.reset();
}

}  // namespace openvmp_control_interactive