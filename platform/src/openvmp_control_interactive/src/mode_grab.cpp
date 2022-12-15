/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-12-10
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "openvmp_control_interactive/mode_grab.hpp"

namespace openvmp_control_interactive {

void GrabMode::enter(std::shared_ptr<ControlImpl> from) {
  ControlImpl::enter(from);

  RCLCPP_DEBUG(node_->get_logger(), "GrabMode::enter()");

  // Prepare markers
  {
    visualization_msgs::msg::InteractiveMarker interactive_marker;
    interactive_marker.header.frame_id = "base_link";
    interactive_marker.name = "openvmp_grab_x";
    interactive_marker.description = "move controller for base_link";
    interactive_marker.scale = 1.6;

    visualization_msgs::msg::InteractiveMarkerControl control;
    control.name = "move_x";
    control.orientation_mode =
        visualization_msgs::msg::InteractiveMarkerControl::FIXED;
    control.interaction_mode =
        visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
    control.orientation.w = 1.0;
    control.orientation.x = 1.0;
    control.orientation.y = 0;
    control.orientation.z = 0;
    interactive_marker.controls.push_back(control);

    visualization_msgs::msg::InteractiveMarkerControl control_z;
    control_z.name = "move_z";
    control_z.orientation_mode =
        visualization_msgs::msg::InteractiveMarkerControl::FIXED;
    control_z.interaction_mode =
        visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
    control_z.orientation.w = 1.0;
    control_z.orientation.x = 0;
    control_z.orientation.y = 1.0;
    control_z.orientation.z = 0;
    interactive_marker.controls.push_back(control_z);

    server_->insert(interactive_marker);
    server_->setCallback(
        interactive_marker.name,
        std::bind(&GrabMode::processFeedback_, this, std::placeholders::_1));
  }
  RCLCPP_DEBUG(node_->get_logger(), "GrabMode::enter(): markers are ready");

  // Prepare the templates
  {
    msg_template_.joint_names = {
        "front_turn_table_joint", "front_body_joint",
        "front_left_arm_joint",   "front_left_arm_inner_joint",
        "front_right_arm_joint",  "front_right_arm_inner_joint",
        "rear_turn_table_joint",  "rear_body_joint",
        "rear_left_arm_joint",    "rear_left_arm_inner_joint",
        "rear_right_arm_joint",   "rear_right_arm_inner_joint"};
    point_template_.time_from_start.sec = 0;
    point_template_.time_from_start.nanosec = 500000000ULL;
    point_template_.positions = {1.57, 1.57, 3.26, -2.3, -1.4, 0.75,
                                 0,    1.57, 3.5,  0.75, 3.5,  0.75};
  }
  RCLCPP_DEBUG(node_->get_logger(), "GrabMode::enter(): templates are ready");

  // Move into the initial position
  {
    trajectory_msgs::msg::JointTrajectory msg = msg_template_;
    trajectory_msgs::msg::JointTrajectoryPoint point = point_template_;
    msg.points.push_back(point);
    if (trajectory_commands_) {
      trajectory_commands_->publish(msg);
    }
  }
  RCLCPP_DEBUG(node_->get_logger(),
               "GrabMode::enter(): moved into the initial position");
}

void GrabMode::leave(std::shared_ptr<ControlImpl> to) {
  RCLCPP_DEBUG(node_->get_logger(), "GrabMode::leave()");
  if (!server_->setCallback("openvmp_grab_x", nullptr)) {
    RCLCPP_ERROR(node_->get_logger(), "setCallback(nullptr) failed");
  }
  if (!server_->erase("openvmp_grab_x")) {
    RCLCPP_ERROR(node_->get_logger(), "erase() failed");
  }
  server_->applyChanges();

  ControlImpl::leave(to);
}

void GrabMode::processFeedback_(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr
        &feedback) {
  if (feedback->event_type !=
      visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_UP) {
    // Did not commit to the action yet.
    return;
  }

  if (feedback->control_name == "move_x") {
    double dist = feedback->pose.position.x;
    RCLCPP_INFO(node_->get_logger(), "Lean %.02f", dist);

    lean_ -= feedback->pose.position.x;
    if (lean_ < LEAN_LIMIT_BOTTOM) lean_ = LEAN_LIMIT_BOTTOM;
    if (lean_ > LEAN_LIMIT_TOP) lean_ = LEAN_LIMIT_TOP;
    RCLCPP_INFO(node_->get_logger(), "Lean at %.02f", lean_);

    trajectory_msgs::msg::JointTrajectory msg = msg_template_;
    trajectory_msgs::msg::JointTrajectoryPoint point = point_template_;
    for (int i : {0}) {
      point.positions[i] -= turn_;
    }
    for (int i : {4, 8, 10}) {
      point.positions[i] += lean_;
    }
    msg.points.push_back(point);
    if (trajectory_commands_) {
      trajectory_commands_->publish(msg);
    }
  } else if (feedback->control_name == "move_z") {
    RCLCPP_INFO(node_->get_logger(), "Turn %.02f", feedback->pose.position.z);

    turn_ -= feedback->pose.position.z;
    if (turn_ < TURN_LIMIT_BOTTOM) turn_ = TURN_LIMIT_BOTTOM;
    if (turn_ > TURN_LIMIT_TOP) turn_ = TURN_LIMIT_TOP;
    RCLCPP_INFO(node_->get_logger(), "Turn at %.02f", turn_);

    trajectory_msgs::msg::JointTrajectory msg = msg_template_;
    trajectory_msgs::msg::JointTrajectoryPoint point = point_template_;
    for (int i : {0}) {
      point.positions[i] -= turn_;
    }
    for (int i : {4, 8, 10}) {
      point.positions[i] += lean_;
    }
    msg.points.push_back(point);
    if (trajectory_commands_) {
      trajectory_commands_->publish(msg);
    }
  }

  server_->setPose("openvmp_grab_x", geometry_msgs::msg::Pose());
  server_->applyChanges();
}

}  // namespace openvmp_control_interactive
