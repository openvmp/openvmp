/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-10-25
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_CONTROL_INTERACTIVE_NODE_H
#define OPENVMP_CONTROL_INTERACTIVE_NODE_H

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "interactive_markers/interactive_marker_server.hpp"
#include "interactive_markers/menu_handler.hpp"
#include "interactive_markers/tools.hpp"
#include "openvmp_control_interactive/link.hpp"
#include "openvmp_control_interactive/mode.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "visualization_msgs/msg/interactive_marker.hpp"
#include "visualization_msgs/msg/interactive_marker_control.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace openvmp_control_interactive {

class Node : public rclcpp::Node {
 public:
  Node();
  ~Node() = default;

 private:
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr
      trajectory_commands_;
  // rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
  //     position_commands_;
  // rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
  //     velocity_commands_;
  std::unique_ptr<interactive_markers::InteractiveMarkerServer> server_;
  std::unique_ptr<interactive_markers::MenuHandler> menu_handler_;
  std::map<interactive_markers::MenuHandler::EntryHandle,
           std::shared_ptr<ModeImpl>>
      modes_;

  interactive_markers::MenuHandler::EntryHandle item_last_;
  Mode mode_last_;

  void initMenu_(std::vector<std::shared_ptr<ModeImpl>>);
  void processFeedback_(
      const std::string &marker_name, const std::string &link_name,
      const Link &link,
      const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr
          &feedback);
  void modeCb_(
      const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr
          &feedback);
};

}  // namespace openvmp_control_interactive

#endif  // OPENVMP_CONTROL_INTERACTIVE_NODE_H
