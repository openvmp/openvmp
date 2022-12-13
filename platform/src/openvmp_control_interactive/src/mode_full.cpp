/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-12-10
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "openvmp_control_interactive/mode_full.hpp"

#include "openvmp_control_interactive/link.hpp"

namespace openvmp_control_interactive {

FullMode::FullMode(
    rclcpp::Node *node,
    std::shared_ptr<interactive_markers::InteractiveMarkerServer> server)
    : TrajVelControl(node, server, FULL, "Full control") {
  state_lock_.lock();
  trajectory_state_subscription_ = node->create_subscription<
      control_msgs::msg::JointTrajectoryControllerState>(
      node_->get_effective_namespace() + "/trajectory_controller/state", 1,
      std::bind(&FullMode::trajectoryStateHandler_, this,
                std::placeholders::_1));
  state_lock_.unlock();
}

FullMode::~FullMode() {
  // TODO(clairbee): decide what needs to be cleaned up properly
}

void FullMode::enter(std::shared_ptr<ControlImpl> from) {
  ControlImpl::enter(from);

  RCLCPP_DEBUG(node_->get_logger(), "FullMode::enter()");

  state_lock_.lock();
  // Create joint controls
  const auto &links = get_links();
  for (auto link_it = links.cbegin(); link_it != links.cend(); link_it++) {
    const auto &link_name = link_it->first;
    const auto &link = link_it->second;

    RCLCPP_DEBUG(node_->get_logger(), "Creating link controls for %s",
                 link_name.c_str());

    visualization_msgs::msg::InteractiveMarker interactive_marker;
    interactive_marker.header.frame_id = link_name;
    interactive_marker.name = "openvmp_twist_marker_" + link_name;
    interactive_marker.description = "twist controller for " + link_name;
    interactive_marker.scale = link.marker_size_scale;

    visualization_msgs::msg::InteractiveMarkerControl control;
    control.orientation_mode =
        visualization_msgs::msg::InteractiveMarkerControl::FIXED;
    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 0;

    if (link.mode ==
        visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS) {
      control.interaction_mode =
          visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
      control.name = "move_";
    } else if (link.mode ==
               visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS) {
      control.interaction_mode =
          visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
      control.name = "rotate_";
    } else {
      RCLCPP_ERROR(node_->get_logger(), "%s(%d): %s", __FILE__, __LINE__,
                   "invalid mode");
    }

    switch (link.axis_vis) {
      case Link::X:
        control.orientation.x = 1;
        control.name += "x";
        break;
      case Link::Y:
        control.orientation.y = 1;
        control.name += "y";
        break;
      case Link::Z:
        control.orientation.z = 1;
        control.name += "z";
        break;
    }

    interactive_marker.controls.push_back(control);

    server_->insert(interactive_marker);
    server_->setCallback(
        interactive_marker.name,
        std::bind(&FullMode::processFeedback_, this, interactive_marker.name,
                  link_name, link, std::placeholders::_1));
  }
  state_lock_.unlock();
  RCLCPP_DEBUG(node_->get_logger(), "FullMode::enter(): done");
}

void FullMode::leave(std::shared_ptr<ControlImpl> to) {
  RCLCPP_DEBUG(node_->get_logger(), "FullMode::leave()");

  state_lock_.lock();
  const auto &links = get_links();
  for (auto link_it = links.cbegin(); link_it != links.cend(); link_it++) {
    const auto &link_name = link_it->first;
    if (!server_->setCallback("openvmp_twist_marker_" + link_name, nullptr)) {
      RCLCPP_ERROR(node_->get_logger(), "setCallback(nullptr) failed for %s",
                   link_name.c_str());
    }
    if (!server_->erase("openvmp_twist_marker_" + link_name)) {
      RCLCPP_ERROR(node_->get_logger(), "erase() failed for %s",
                   link_name.c_str());
    }
  }
  server_->applyChanges();
  state_lock_.unlock();

  ControlImpl::leave(to);
}

void FullMode::trajectoryStateHandler_(
    const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg) {
  if (state_lock_.try_lock()) {
    int size = msg->joint_names.size();
    std::map<std::string, double> positions;
    for (int i = 0; i < size; i++) {
      positions.insert({msg->joint_names[i], msg->actual.positions[i]});
    }

    auto &links = get_links();
    for (auto link_it = links.begin(); link_it != links.end(); link_it++) {
      link_it->second.last_angle = positions[link_it->second.joint];
    }
    state_lock_.unlock();
  }
}

void FullMode::processFeedback_(
    const std::string &marker_name, const std::string &link_name,
    const Link &link,
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr
        &feedback) {
  // geometry_msgs::msg::Twist vel_msg;
  // double velocity = 0;

  // if (link.mode ==
  //     visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS) {
  //   switch (link.axis) {
  //     case Link::X:
  //       vel_msg.linear.x = link.linear_drive_scale *
  //       feedback->pose.position.x; vel_msg.linear.x =
  //       std::min(vel_msg.linear.x, link.max_linear_velocity);
  //       vel_msg.linear.x =
  //           std::max(vel_msg.linear.x, -link.max_linear_velocity);
  //       velocity = vel_msg.linear.x;
  //       break;
  //     case Link::Y:
  //       vel_msg.linear.y = link.linear_drive_scale *
  //       feedback->pose.position.y; vel_msg.linear.y =
  //       std::min(vel_msg.linear.y, link.max_linear_velocity);
  //       vel_msg.linear.y =
  //           std::max(vel_msg.linear.y, -link.max_linear_velocity);

  //       velocity = vel_msg.linear.y;
  //       break;
  //     case Link::Z:
  //       vel_msg.linear.z = link.linear_drive_scale *
  //       feedback->pose.position.z; vel_msg.linear.z =
  //       std::min(vel_msg.linear.z, link.max_linear_velocity);
  //       vel_msg.linear.z =
  //           std::max(vel_msg.linear.z, -link.max_linear_velocity);
  //       velocity = vel_msg.linear.z;
  //       break;
  //   }
  // } else if (link.mode ==
  //            visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS)
  //            {
  //   double roll, pitch, yaw;
  //   tf2::Quaternion quat_tf;
  //   tf2::fromMsg(feedback->pose.orientation, quat_tf);
  //   tf2::Matrix3x3(quat_tf).getRPY(roll, pitch, yaw);

  //   switch (link.axis) {
  //     case Link::X:
  //       vel_msg.angular.x = link.angular_drive_scale * roll;
  //       vel_msg.angular.x =
  //           std::min(vel_msg.angular.x, link.max_angular_velocity);
  //       vel_msg.angular.x =
  //           std::max(vel_msg.angular.x, -link.max_angular_velocity);
  //       velocity = vel_msg.angular.x;
  //       break;
  //     case Link::Y:
  //       vel_msg.angular.y = link.angular_drive_scale * pitch;
  //       vel_msg.angular.y =
  //           std::min(vel_msg.angular.y, link.max_angular_velocity);
  //       vel_msg.angular.y =
  //           std::max(vel_msg.angular.y, -link.max_angular_velocity);
  //       velocity = vel_msg.angular.y;
  //       break;
  //     case Link::Z:
  //       vel_msg.angular.z = link.angular_drive_scale * yaw;
  //       vel_msg.angular.z =
  //           std::min(vel_msg.angular.z, link.max_angular_velocity);
  //       vel_msg.angular.z =
  //           std::max(vel_msg.angular.z, -link.max_angular_velocity);
  //       velocity = vel_msg.angular.z;
  //       break;
  //   }
  // } else {
  //   RCLCPP_ERROR(get_logger(), "%s(%d): %s", __FILE__, __LINE__,
  //                "invalid mode");
  // }

  // FIXME(clairbee): NOT YET
  // vel_pubs_[link_name]->publish(vel_msg);

  if (!feedback ||
      feedback->event_type !=
          visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_UP) {
    return;
  }

  state_lock_.lock();
  RCLCPP_DEBUG(node_->get_logger(), "FullMode::processFeedback_()");

  // trajectory controller
  trajectory_msgs::msg::JointTrajectory msg;
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.time_from_start.sec = 0;
  point.time_from_start.nanosec = 500000000ULL;

  auto &links = get_links();
  for (auto link_it = links.begin(); link_it != links.end(); link_it++) {
    const auto &that_link_name = link_it->first;
    auto &that_link = link_it->second;

    msg.joint_names.push_back(that_link.joint);

    if (that_link_name == link_name) {
      double angle = 0.0;
      switch (link.axis_rot) {
        case Link::X:
          angle = feedback->pose.orientation.x;
          break;
        case Link::Y:
          angle = feedback->pose.orientation.y;
          break;
        case Link::Z:
          angle = feedback->pose.orientation.z;
          break;
      }
      if (link.invert) {
        angle = -angle;
      }
      that_link.last_angle += angle;
    }
    point.positions.push_back(that_link.last_angle);
  }
  msg.points.push_back(point);
  if (trajectory_commands_) {
    trajectory_commands_->publish(msg);
  }

  // // position controller
  // std_msgs::msg::Float64MultiArray msg;
  // msg.data.resize(LINKS_TOTAL);
  //
  // switch (link.axis_rot) {
  //   case Link::X:
  //     msg.data[link.index] = feedback->pose.orientation.x;
  //     break;
  //   case Link::Y:
  //     msg.data[link.index] = feedback->pose.orientation.y;
  //     break;
  //   case Link::Z:
  //     msg.data[link.index] = feedback->pose.orientation.z;
  //     break;
  // }
  // position_commands_->publish(msg);

  // velocity controller
  // std_msgs::msg::Float64MultiArray msg;
  // msg.data.resize(LINKS_TOTAL);
  //
  // msg.data[link.index] = velocity;
  // velocity_commands_->publish(msg);

  // Make the marker snap back to robot
  server_->setPose(marker_name, geometry_msgs::msg::Pose());
  server_->applyChanges();

  state_lock_.unlock();
}

}  // namespace openvmp_control_interactive
