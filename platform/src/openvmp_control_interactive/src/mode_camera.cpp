/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-12-17
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "openvmp_control_interactive/mode_camera.hpp"

#include "openvmp_control_interactive/link.hpp"

namespace openvmp_control_interactive {

const std::vector<std::string> CamerasMode::joints_ = {
    "front_left_arm_camera_servo_upper_link",  "front_left_arm_camera",
    "front_right_arm_camera_servo_upper_link", "front_right_arm_camera",
    "rear_left_arm_camera_servo_upper_link",   "rear_left_arm_camera",
    "rear_right_arm_camera_servo_upper_link",  "rear_right_arm_camera",
};

std::map<std::string, double> CamerasMode::joints_state_ = {
    {"front_left_arm_camera_servo_upper_link", 0.0},
    {"front_left_arm_camera", 0.0},
    {"front_right_arm_camera_servo_upper_link", 0.0},
    {"front_right_arm_camera", 0.0},
    {"rear_left_arm_camera_servo_upper_link", 0.0},
    {"rear_left_arm_camera", 0.0},
    {"rear_right_arm_camera_servo_upper_link", 0.0},
    {"rear_right_arm_camera", 0.0},
};

static const std::map<Mode, const std::string> PREFIXES = {
    {CAMERAS_FRONT_LEFT, "front_left"},
    {CAMERAS_FRONT_RIGHT, "front_right"},
    {CAMERAS_REAR_LEFT, "rear_left"},
    {CAMERAS_REAR_RIGHT, "rear_right"},
};

CamerasMode::CamerasMode(
    rclcpp::Node *node,
    std::shared_ptr<interactive_markers::InteractiveMarkerServer> server,
    Mode m)
    : PositionControl(node, server, m, "Cameras " + PREFIXES.at(m)),
      prefix_{PREFIXES.at(m)} {}

void CamerasMode::enter(std::shared_ptr<ControlImpl> from) {
  ControlImpl::enter(from);

  RCLCPP_DEBUG(node_->get_logger(), "CamerasMode::enter()");

  const std::vector<std::string> link_names = {
      prefix_ + "_arm_camera_servo_upper_link",
      prefix_ + "_arm_camera",
  };

  for (auto link_name : link_names) {
    visualization_msgs::msg::InteractiveMarker interactive_marker;
    interactive_marker.header.frame_id = link_name;
    interactive_marker.name = "openvmp_marker_" + link_name;
    interactive_marker.description = "twist controller for " + link_name;
    interactive_marker.scale = 0.25;

    visualization_msgs::msg::InteractiveMarkerControl control;
    control.orientation_mode =
        visualization_msgs::msg::InteractiveMarkerControl::FIXED;
    control.interaction_mode =
        visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
    control.orientation.w = 1;
    if (link_name == link_names[0]) {
      control.orientation.x = 0;
      control.orientation.y = 1;
      control.orientation.z = 0;
    } else {
      // control.orientation.y = sin(M_PI / 6.0) / cos(M_PI / 6.0);
      control.orientation.x = 1;
      control.orientation.y = 0;
      control.orientation.z = 0;
    }
    control.name = "rotate_x";

    interactive_marker.controls.push_back(control);

    server_->insert(interactive_marker);
    server_->setCallback(interactive_marker.name,
                         std::bind(&CamerasMode::processFeedback_, this,
                                   link_name, std::placeholders::_1));
  }
}

void CamerasMode::leave(std::shared_ptr<ControlImpl> to) {
  RCLCPP_DEBUG(node_->get_logger(), "CamerasMode::leave()");

  ControlImpl::leave(to);
}

void CamerasMode::processFeedback_(
    const std::string &link_name,
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr
        &feedback) {
  if (!feedback ||
      feedback->event_type !=
          visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_UP) {
    return;
  }

  RCLCPP_INFO(node_->get_logger(), "CamerasMode::processFeedback_()");

  // The lower servo produces the 'z' component.
  // The upper servo produces the 'x' component.
  joints_state_[link_name] +=
      feedback->pose.orientation.x + feedback->pose.orientation.z;

  std_msgs::msg::Float64MultiArray msg;
  msg.data.resize(joints_.size());
  for (size_t i = 0; i < joints_.size(); i++) {
    msg.data[i] = joints_state_[joints_[i]];
    RCLCPP_INFO(node_->get_logger(), "CamerasMode::processFeedback_(): %.02f",
                msg.data[i]);
  }
  position_commands_->publish(msg);

  // Make the marker snap back to robot
  server_->setPose(feedback->marker_name, geometry_msgs::msg::Pose());
  server_->applyChanges();
}

}  // namespace openvmp_control_interactive
