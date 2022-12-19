/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-12-12
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "openvmp_control_interactive/mode_drive.hpp"

namespace openvmp_control_interactive {

void DriveMode::enter(std::shared_ptr<ControlImpl> from) {
  ControlImpl::enter(from);

  RCLCPP_DEBUG(node_->get_logger(), "DriveMode::enter()");

  // Prepare markers
  {
    visualization_msgs::msg::InteractiveMarker interactive_marker;
    interactive_marker.header.frame_id = "base_link";
    interactive_marker.name = "openvmp_drive_x";
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

    visualization_msgs::msg::InteractiveMarkerControl control_turn;
    control_turn.name = "turn";
    control_turn.orientation_mode =
        visualization_msgs::msg::InteractiveMarkerControl::FIXED;
    control_turn.interaction_mode =
        visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
    control_turn.orientation.w = 1.0;
    control_turn.orientation.x = 0;
    control_turn.orientation.y = 0.0;
    control_turn.orientation.z = 1.0;
    interactive_marker.controls.push_back(control_turn);

    server_->insert(interactive_marker);
    server_->setCallback(
        interactive_marker.name,
        std::bind(&DriveMode::processFeedback_, this, std::placeholders::_1));
  }
  RCLCPP_DEBUG(node_->get_logger(), "DriveMode::enter(): markers are ready");

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
    point_template_.positions = {0, 0, 3.14, -0.75, 3.14, -0.75,
                                 0, 0, 3.14, -0.75, 3.14, -0.75};
  }
  RCLCPP_DEBUG(node_->get_logger(), "DriveMode::enter(): templates are ready");

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
               "DriveMode::enter(): moved into the initial position");
}

void DriveMode::leave(std::shared_ptr<ControlImpl> to) {
  RCLCPP_DEBUG(node_->get_logger(), "DriveMode::leave()");
  if (!server_->setCallback("openvmp_drive_x", nullptr)) {
    RCLCPP_ERROR(node_->get_logger(), "setCallback(nullptr) failed");
  }
  if (!server_->erase("openvmp_drive_x")) {
    RCLCPP_ERROR(node_->get_logger(), "erase() failed");
  }
  server_->applyChanges();

  ControlImpl::leave(to);
}

void DriveMode::processFeedback_(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr
        &feedback) {
  if (feedback->event_type !=
      visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_UP) {
    // Did not commit to the action yet.
    return;
  }

  if (feedback->control_name == "move_x") {
    double dist = feedback->pose.position.x;
    RCLCPP_INFO(node_->get_logger(), "Drive %.02f", dist);

    std_msgs::msg::Float64MultiArray msg;
    msg.data = {dist, dist, dist, dist};
    if (velocity_commands_) {
      velocity_commands_->publish(msg);
    }
  } else if (feedback->control_name == "move_z") {
    RCLCPP_INFO(node_->get_logger(), "Lift on %.02f",
                feedback->pose.position.z);

    lift_ += feedback->pose.position.z;
    if (lift_ < LIFT_LIMIT_BOTTOM) lift_ = LIFT_LIMIT_BOTTOM;
    if (lift_ > LIFT_LIMIT_TOP) lift_ = LIFT_LIMIT_TOP;
    RCLCPP_INFO(node_->get_logger(), "Lift at %.02f", lift_);

    trajectory_msgs::msg::JointTrajectory msg = msg_template_;
    trajectory_msgs::msg::JointTrajectoryPoint point = point_template_;
    for (int i : {3, 5, 9, 11}) {
      point.positions[i] -= lift_;
    }
    msg.points.push_back(point);
    if (trajectory_commands_) {
      trajectory_commands_->publish(msg);
    }
  } else if (feedback->control_name == "turn") {
    RCLCPP_INFO(node_->get_logger(), "Turn on %.02f",
                feedback->pose.position.y);

    turn_ -= feedback->pose.position.y;
    if (turn_ < TURN_LIMIT_BOTTOM) turn_ = TURN_LIMIT_BOTTOM;
    if (turn_ > TURN_LIMIT_TOP) turn_ = TURN_LIMIT_TOP;
    RCLCPP_INFO(node_->get_logger(), "Turn at %.02f", lift_);

    trajectory_msgs::msg::JointTrajectory msg = msg_template_;
    trajectory_msgs::msg::JointTrajectoryPoint point = point_template_;
    point.positions[0] += turn_;
    point.positions[6] -= turn_;
    msg.points.push_back(point);
    if (trajectory_commands_) {
      trajectory_commands_->publish(msg);
    }
  }

  server_->setPose("openvmp_drive_x", geometry_msgs::msg::Pose());
  server_->applyChanges();
}

}  // namespace openvmp_control_interactive
