/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-10-25
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "openvmp_control_interactive/node.hpp"

#include <algorithm>

#include "openvmp_control_interactive/link.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace openvmp_control_interactive {

Node::Node()
    : rclcpp::Node::Node("openvmp_control_interactive"),
      server_(std::make_unique<interactive_markers::InteractiveMarkerServer>(
          "twist_server", get_node_base_interface(), get_node_clock_interface(),
          get_node_logging_interface(), get_node_topics_interface(),
          get_node_services_interface())) {
  RCLCPP_INFO(this->get_logger(), "Namespace: %s",
              this->get_effective_namespace().c_str());

  position_commands_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      this->get_effective_namespace() + "/position_controller/commands", 10);
  // velocity_commands_ = create_publisher<std_msgs::msg::Float64MultiArray>(
  //     this->get_effective_namespace() + "/velocity_controller/commands", 10);

  const auto &links = get_links();
  for (auto link_it = links.cbegin(); link_it != links.cend(); link_it++) {
    const auto &link_name = link_it->first;
    const auto &link = link_it->second;

    // FIXME(clairbee): NOT YET
    // vel_pubs_.insert({link_name, create_publisher<geometry_msgs::msg::Twist>(
    //                                  "cmd/" + link_name + "/vel", 1)});

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
      RCLCPP_ERROR(get_logger(), "%s(%d): %s", __FILE__, __LINE__,
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
        std::bind(&Node::processFeedback_, this, interactive_marker.name,
                  link_name, link, std::placeholders::_1));
  }
  server_->applyChanges();
}

void Node::processFeedback_(
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
  //       vel_msg.linear.x = link.linear_drive_scale * feedback->pose.position.x;
  //       vel_msg.linear.x = std::min(vel_msg.linear.x, link.max_linear_velocity);
  //       vel_msg.linear.x =
  //           std::max(vel_msg.linear.x, -link.max_linear_velocity);
  //       velocity = vel_msg.linear.x;
  //       break;
  //     case Link::Y:
  //       vel_msg.linear.y = link.linear_drive_scale * feedback->pose.position.y;
  //       vel_msg.linear.y = std::min(vel_msg.linear.y, link.max_linear_velocity);
  //       vel_msg.linear.y =
  //           std::max(vel_msg.linear.y, -link.max_linear_velocity);

  //       velocity = vel_msg.linear.y;
  //       break;
  //     case Link::Z:
  //       vel_msg.linear.z = link.linear_drive_scale * feedback->pose.position.z;
  //       vel_msg.linear.z = std::min(vel_msg.linear.z, link.max_linear_velocity);
  //       vel_msg.linear.z =
  //           std::max(vel_msg.linear.z, -link.max_linear_velocity);
  //       velocity = vel_msg.linear.z;
  //       break;
  //   }
  // } else if (link.mode ==
  //            visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS) {
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


  (void)link_name;
  // FIXME(clairbee): NOT YET
  // vel_pubs_[link_name]->publish(vel_msg);
  std_msgs::msg::Float64MultiArray msg;
  msg.data.resize(LINKS_TOTAL);

  // position controller
  switch (link.axis_rot) {
    case Link::X:
      msg.data[link.index] = feedback->pose.orientation.x;
      break;
    case Link::Y:
      msg.data[link.index] = feedback->pose.orientation.y;
      break;
    case Link::Z:
      msg.data[link.index] = feedback->pose.orientation.z;
      break;
  }
  position_commands_->publish(msg);

  // velocity controller
  // msg.data[link.index] = velocity;
  // velocity_commands_->publish(msg);

  // Make the marker snap back to robot
  server_->setPose(marker_name, geometry_msgs::msg::Pose());
  server_->applyChanges();
}

}  // namespace openvmp_control_interactive
