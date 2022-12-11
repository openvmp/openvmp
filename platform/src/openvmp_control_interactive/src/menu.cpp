/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-12-10
 *
 * Licensed under Apache License, Version 2.0.
 */

#include <algorithm>

#include "builtin_interfaces/msg/duration.hpp"
#include "openvmp_control_interactive/link.hpp"
#include "openvmp_control_interactive/node.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace openvmp_control_interactive {

void Node::initMenu_(std::vector<std::shared_ptr<ModeImpl>> modes) {
  std::string mode = "Mode";

  menu_handler_ = std::make_unique<interactive_markers::MenuHandler>();
  interactive_markers::MenuHandler::EntryHandle mode_menu_handle =
      menu_handler_->insert(mode);

  mode_last_ = DEFAULT;
  for (auto &mode : modes) {
    auto item = menu_handler_->insert(
        mode_menu_handle, mode->get_name(),
        std::bind(&Node::modeCb_, this, std::placeholders::_1));
    modes_.insert({item, mode});

    if (mode->get_mode() == mode_last_) {
      item_last_ = item;
      menu_handler_->setCheckState(item,
                                   interactive_markers::MenuHandler::CHECKED);
    } else {
      menu_handler_->setCheckState(item,
                                   interactive_markers::MenuHandler::UNCHECKED);
    }
  }

  float scale = 1.0;

  visualization_msgs::msg::Marker box_marker;
  box_marker.type = visualization_msgs::msg::Marker::CYLINDER;
  box_marker.scale.x = scale * 0.50;
  box_marker.scale.y = scale * 0.50;
  box_marker.scale.z = scale * 0.02;
  box_marker.color.r = 0.7;
  box_marker.color.g = 0.7;
  box_marker.color.b = 0.0;
  box_marker.color.a = 1.0;

  visualization_msgs::msg::InteractiveMarkerControl control;
  control.interaction_mode =
      visualization_msgs::msg::InteractiveMarkerControl::MENU;
  control.always_visible = true;
  control.markers.push_back(box_marker);

  visualization_msgs::msg::InteractiveMarker int_marker;
  int_marker.header.frame_id = "base_footprint";
  int_marker.pose.position.y = 0;
  int_marker.scale = scale;
  int_marker.name = mode;
  int_marker.controls.push_back(control);

  server_->insert(int_marker);
  menu_handler_->apply(*server_, mode);
}

void Node::modeCb_(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr
        &feedback) {
  auto item_new = feedback->menu_entry_id;
  auto mode_new = modes_[item_new]->get_mode();

  // Unheck the menu item of the old mode
  RCLCPP_INFO(get_logger(), "Old mode item: %d", item_last_);
  menu_handler_->setCheckState(item_last_,
                               interactive_markers::MenuHandler::UNCHECKED);

  // Check the menu item of the new mode
  RCLCPP_INFO(get_logger(), "New mode item: %d", item_new);
  RCLCPP_INFO(get_logger(), "New mode: %d", mode_new);
  menu_handler_->setCheckState(item_new,
                               interactive_markers::MenuHandler::CHECKED);

  // Perform the transition
  modes_[item_last_]->leave(mode_new);
  modes_[item_new]->enter(mode_last_);
  item_last_ = item_new;
  mode_last_ = mode_new;

  menu_handler_->reApply(*server_);
  server_->applyChanges();
}

}  // namespace openvmp_control_interactive
