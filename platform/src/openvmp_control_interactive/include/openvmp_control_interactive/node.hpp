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

#include "interactive_markers/interactive_marker_server.hpp"
#include "interactive_markers/menu_handler.hpp"
#include "openvmp_control_interactive/control.hpp"
#include "openvmp_control_interactive/mode.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/interactive_marker.hpp"
#include "visualization_msgs/msg/interactive_marker_control.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace openvmp_control_interactive {

class Node : public rclcpp::Node {
 public:
  Node();
  ~Node() = default;

 private:
  Modes modes_;
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
  std::unique_ptr<interactive_markers::MenuHandler> menu_handler_;
  std::map<interactive_markers::MenuHandler::EntryHandle,
           std::shared_ptr<ControlImpl>>
      menu_modes_;

  interactive_markers::MenuHandler::EntryHandle item_last_;
  Mode mode_last_;

  void initMenu_();
  void modeCb_(
      const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr
          &feedback);
};

}  // namespace openvmp_control_interactive

#endif  // OPENVMP_CONTROL_INTERACTIVE_NODE_H
