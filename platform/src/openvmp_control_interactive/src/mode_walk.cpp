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
  }
  RCLCPP_DEBUG(node_->get_logger(), "WalkMode::enter(): templates are ready");

  // Move into the initial position
  {
    trajectory_msgs::msg::JointTrajectory msg = msg_template_;
    trajectory_msgs::msg::JointTrajectoryPoint point;
    builtin_interfaces::msg::Duration no_delay;
    next_phase_(0, point, no_delay);
    msg.points.push_back(point);
    if (trajectory_commands_) {
      trajectory_commands_->publish(msg);
    }
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
  if (feedback->event_type !=
      visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_UP) {
    // Did not commit to the action yet.
    return;
  }

  if (feedback->control_name == "move_x") {
    double dist = feedback->pose.position.x;
    RCLCPP_INFO(node_->get_logger(), "Walk %.02f", dist);

    trajectory_msgs::msg::JointTrajectory msg = msg_template_;
    builtin_interfaces::msg::Duration no_delay;
    next_phase_animation_(dist > 0 ? 1 : -1, msg, no_delay);

    if (trajectory_commands_) {
      trajectory_commands_->publish(msg);
    }
  } else if (feedback->control_name == "move_z") {
    RCLCPP_INFO(node_->get_logger(), "Lift on %.02f",
                feedback->pose.position.z);

    lift_ += feedback->pose.position.z;
    if (lift_ < LIFT_LIMIT_BOTTOM) lift_ = LIFT_LIMIT_BOTTOM;
    if (lift_ > LIFT_LIMIT_TOP) lift_ = LIFT_LIMIT_TOP;
    RCLCPP_INFO(node_->get_logger(), "Lift at %.02f", lift_);

    trajectory_msgs::msg::JointTrajectory msg = msg_template_;
    trajectory_msgs::msg::JointTrajectoryPoint point;
    next_phase_(0, point, point.time_from_start);

    msg.points.push_back(point);

    if (trajectory_commands_) {
      trajectory_commands_->publish(msg);
    }
  }

  server_->setPose("openvmp_walk_x", geometry_msgs::msg::Pose());
  server_->applyChanges();
}

void WalkMode::next_phase_(
    int dir, trajectory_msgs::msg::JointTrajectoryPoint &point,
    const builtin_interfaces::msg::Duration &time_from_start) {
  point.time_from_start = time_from_start;
  point.time_from_start.nanosec += 500000000ULL;

  phase_ += dir;
  phase_ %= 4;

  // TODO(clairbee): merge the phases below to form 8 phases, animate to even
  //                 phases only (when all 4 are on the ground)

  // // Phase 1: (0.5)
  // //  XOOO OOOX
  // //  OOXO OXOO
  // // Phase 2: (1.5)
  // //  OXOO XOOO
  // //  OOOX OOXO
  // // Phase 3: (2.5)
  // //  OOXO OXOO
  // //  XOOO OOOX
  // // Phase 4: (3.5)
  // //  OOOX OOXO
  // //  OXOO XOOO
  // // where:
  // //  The first two positions are mirrors of the second two.
  // //  The first position is 0.5 angle, 0 lift.
  // //  The second position is 0.25 angle, 0.1 lift.
  // const double a1 = 0.5, a2 = 0.25;
  // const double l_base = 0.7 + lift_,  // the base lift
  //     l1 = l_base, l2 = l_base - 0.1;
  // switch (phase_) {
  //   case 0:
  //     point.positions = {0, 0, -a2, l2, a1, l1, 0, 0, -a2, l2, a1, l1};
  //     break;
  //   case 1:
  //     point.positions = {0, 0, -a1, l1, a2, l2, 0, 0, a2, l2, -a1, l1};
  //     break;
  //   case 2:
  //     point.positions = {0, 0, a1, l1, -a2, l2, 0, 0, a1, l1, -a2, l2};
  //     break;
  //   case 3:
  //     point.positions = {0, 0, a2, l2, -a1, l1, 0, 0, -a1, l1, a2, l2};
  //     break;
  // }

  // Phase 1:
  //  XOO OXO
  //  OOX O-O
  // Phase 2:
  //  OXO OOX
  //  O-O XOO
  // Phase 3:
  //  OOX O-O
  //  XOO OXO
  // Phase 4:
  //  O-O XOO
  //  OXO OOX
  // where:
  //  The first and last positions are mirrors.
  //  The middle position is neutral.
  //  The first position is 0.5 angle, -0.2 lift.
  //  The middle position raised is 0.3 lift.
  const double a1 = 0.6;
  const double l0 = 0.7 + lift_,  // the base lift
      l1 = l0 + 0.3, l2 = l0 - 0.5;
  switch (phase_) {
    case 0:
      point.positions = {0, 0, -a1, l1, a1, l1, 0, 0, 0, l2, 0, l0};
      break;
    case 1:
      point.positions = {0, 0, 0, l2, 0, l0, 0, 0, -a1, l1, a1, l1};
      break;
    case 2:
      point.positions = {0, 0, a1, l1, -a1, l1, 0, 0, 0, l0, 0, l2};
      break;
    case 3:
      point.positions = {0, 0, 0, l0, 0, l2, 0, 0, a1, l1, -a1, l1};
      break;
  }
}

void WalkMode::next_phase_animation_(
    int dir, trajectory_msgs::msg::JointTrajectory &msg,
    const builtin_interfaces::msg::Duration &time_from_start) {
  trajectory_msgs::msg::JointTrajectoryPoint point;

  next_phase_(dir, point, time_from_start);
  msg.points.push_back(point);

  next_phase_(dir, point, point.time_from_start);
  msg.points.push_back(point);

  next_phase_(dir, point, point.time_from_start);
  msg.points.push_back(point);

  next_phase_(dir, point, point.time_from_start);
  msg.points.push_back(point);
}

}  // namespace openvmp_control_interactive
