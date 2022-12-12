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
  ControlImpl::enter(from);

  RCLCPP_DEBUG(node_->get_logger(), "WalkMode::enter()");

  // Prepare markers
  {
    visualization_msgs::msg::InteractiveMarker interactive_marker;
    interactive_marker.header.frame_id = "base_link";
    interactive_marker.name = "openvmp_walk_x";
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
        std::bind(&WalkMode::processFeedback_, this, std::placeholders::_1));
  }
  RCLCPP_DEBUG(node_->get_logger(), "WalkMode::enter(): markers are ready");

  // Prepare the templates
  {
    msg_template_.joint_names = {
        "front_turn_table_joint", "front_body_joint",
        "front_left_arm_joint",   "front_left_arm_inner_joint",
        "front_right_arm_joint",  "front_right_arm_inner_joint",
        "rear_turn_table_joint",  "rear_body_joint",
        "rear_left_arm_joint",    "rear_left_arm_inner_joint",
        "rear_right_arm_joint",   "rear_right_arm_inner_joint"};
    builtin_interfaces::msg::Duration duration;
    duration.sec = 0;
    duration.nanosec = 500000000ULL;
    point_template_.time_from_start = duration;
    point_template_.positions = {0, 0, 0, 0.7, 0, 0.7, 0, 0, 0, 0.7, 0, 0.7};
  }
  RCLCPP_DEBUG(node_->get_logger(), "WalkMode::enter(): templates are ready");

  // Move into the initial position
  {
    trajectory_msgs::msg::JointTrajectory msg = msg_template_;
    trajectory_msgs::msg::JointTrajectoryPoint point = point_template_;
    msg.points.push_back(point);
    trajectory_commands_->publish(msg);
  }
  RCLCPP_DEBUG(node_->get_logger(),
               "WalkMode::enter(): moved into the initial position");
}

void WalkMode::leave(std::shared_ptr<ControlImpl> to) {
  RCLCPP_DEBUG(node_->get_logger(), "WalkMode::leave()");
  if (!server_->setCallback("openvmp_walk_x", nullptr)) {
    RCLCPP_ERROR(node_->get_logger(), "setCallback(nullptr) failed");
  }
  if (!server_->erase("openvmp_walk_x")) {
    RCLCPP_ERROR(node_->get_logger(), "erase() failed");
  }
  server_->applyChanges();

  ControlImpl::leave(to);
}

void WalkMode::processFeedback_(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr
        &feedback) {
  (void)feedback;

  if (feedback->event_type !=
      visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_UP) {
    // Did not commit to the action yet.
    return;
  }

  if (feedback->control_name == "move_x") {
    RCLCPP_INFO(node_->get_logger(), "Walk %.02f", feedback->pose.position.x);

    trajectory_msgs::msg::JointTrajectory msg = msg_template_;
    trajectory_msgs::msg::JointTrajectoryPoint point = point_template_;

    for (int i : {3, 5, 9, 11}) {
      point.positions[i] += lift_;
    }
    point.positions[1] -= 0.1;
    msg.points.push_back(point);

    point.positions[4] = 0.5;
    builtin_interfaces::msg::Duration duration;
    duration.sec = 0;
    duration.nanosec = 1000000000ULL;
    point.time_from_start = duration;
    msg.points.push_back(point);

    point.positions[2] = -0.25;
    point.positions[4] = 0.25;
    point.positions[8] = 0.25;
    point.positions[10] = 0.25;
    duration.sec = 0;
    duration.nanosec = 1500000000ULL;
    point.time_from_start = duration;
    msg.points.push_back(point);

    trajectory_commands_->publish(msg);
  } else if (feedback->control_name == "move_z") {
    RCLCPP_INFO(node_->get_logger(), "Lift on %.02f",
                feedback->pose.position.z);

    trajectory_msgs::msg::JointTrajectory msg = msg_template_;
    trajectory_msgs::msg::JointTrajectoryPoint point = point_template_;

    lift_ -= feedback->pose.position.z;
    if (lift_ < LIFT_LIMIT_BOTTOM) lift_ = LIFT_LIMIT_BOTTOM;
    if (lift_ > LIFT_LIMIT_TOP) lift_ = LIFT_LIMIT_TOP;
    RCLCPP_INFO(node_->get_logger(), "Lift at %.02f", lift_);

    for (int i : {3, 5, 9, 11}) {
      point.positions[i] += lift_;
    }
    msg.points.push_back(point);

    trajectory_commands_->publish(msg);
  }

  server_->setPose("openvmp_walk_x", geometry_msgs::msg::Pose());
  server_->applyChanges();
}

}  // namespace openvmp_control_interactive
