/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-12-17
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "openvmp_control_interactive/control_position.hpp"

#include <algorithm>

namespace openvmp_control_interactive {

std::mutex PositionControl::lock_;
bool PositionControl::initialized_ = false;
rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
    PositionControl::position_commands_ = nullptr;

PositionControl::~PositionControl() {
  lock_.lock();
  if (initialized_) {
    position_commands_.reset();
    initialized_ = false;
  }
  lock_.unlock();
}

void PositionControl::init() {
  RCLCPP_DEBUG(node_->get_logger(), "PositionControl::init()");

  lock_.lock();
  if (!initialized_) {
    RCLCPP_DEBUG(node_->get_logger(), "PositionControl::init(): initializing");
    position_commands_ =
        node_->create_publisher<std_msgs::msg::Float64MultiArray>(
            node_->get_effective_namespace() + "/position_controller/commands",
            10);
    if (!position_commands_) {
      RCLCPP_ERROR(node_->get_logger(),
                   "Failed to subscribe to /position_controller/commands");
    }

    initialized_ = true;
  }
  lock_.unlock();

  RCLCPP_DEBUG(node_->get_logger(), "PositionControl::init(): done");
}

void PositionControl::fini() {
  RCLCPP_DEBUG(node_->get_logger(), "PositionControl::fini()");

  lock_.lock();
  position_commands_.reset();
  initialized_ = false;
  lock_.unlock();
}

}  // namespace openvmp_control_interactive
