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

void Node::initMenu_() {
  const std::string text_control = "Interactive Control";
  const std::string text_body = "Whole Body";
  const std::string text_cameras = "Cameras";

  // Create the memu
  menu_handler_ = std::make_unique<interactive_markers::MenuHandler>();

  // Create the sub-menu for interactive control
  interactive_markers::MenuHandler::EntryHandle control_menu_handle =
      menu_handler_->insert(text_control);

  auto none_mode = modes_[Mode::NONE];
  auto none_item = menu_handler_->insert(
      control_menu_handle, none_mode->get_name(),
      std::bind(&Node::modeCb_, this, std::placeholders::_1));
  menu_handler_->setCheckState(none_item,
                               interactive_markers::MenuHandler::CHECKED);
  menu_modes_.insert({none_item, none_mode});
  mode_last_ = NONE;
  item_last_ = none_item;

  // Create sub-menus for various categories of interactive control
  interactive_markers::MenuHandler::EntryHandle body_menu_handle =
      menu_handler_->insert(control_menu_handle, text_body);
  interactive_markers::MenuHandler::EntryHandle cameras_menu_handle =
      menu_handler_->insert(control_menu_handle, text_cameras);

  // Iterate through all modes and insert them as items
  // into the corresponding sub-menu.
  for (auto &mode : modes_) {
    interactive_markers::MenuHandler::EntryHandle parent_menu_handle;
    if (mode.second->is_whole_body()) {
      parent_menu_handle = body_menu_handle;
    } else if (mode.second->is_cameras()) {
      parent_menu_handle = cameras_menu_handle;
    } else {
      // This mode has no parent menu item?
      // It's probably 'Mode::NONE".
      continue;
    }

    auto item = menu_handler_->insert(
        parent_menu_handle, mode.second->get_name(),
        std::bind(&Node::modeCb_, this, std::placeholders::_1));
    menu_handler_->setCheckState(item,
                                 interactive_markers::MenuHandler::UNCHECKED);
    menu_modes_.insert({item, mode.second});
  }

  float scale = 1.0;

  visualization_msgs::msg::Marker box_marker;
  box_marker.type = visualization_msgs::msg::Marker::CYLINDER;
  box_marker.scale.x = scale * 0.60;
  box_marker.scale.y = scale * 0.60;
  box_marker.scale.z = scale * 0.02;
  box_marker.color.r = 1.0;
  box_marker.color.g = 1.0;
  box_marker.color.b = 0.0;
  box_marker.color.a = 0.7;

  visualization_msgs::msg::InteractiveMarkerControl control;
  control.interaction_mode =
      visualization_msgs::msg::InteractiveMarkerControl::MENU;
  control.always_visible = true;
  control.markers.push_back(box_marker);

  visualization_msgs::msg::InteractiveMarker int_marker;
  int_marker.header.frame_id = "base_footprint";
  int_marker.pose.position.z = 0.9;
  int_marker.scale = scale;
  int_marker.name = text_control;
  int_marker.controls.push_back(control);

  server_->insert(int_marker);
  menu_handler_->apply(*server_, text_control);
}

void Node::modeCb_(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr
        &feedback) {
  if (feedback->event_type !=
      visualization_msgs::msg::InteractiveMarkerFeedback::MENU_SELECT) {
    return;
  }
  auto item_new = feedback->menu_entry_id;
  auto mode_new = menu_modes_[item_new]->get_mode();

  // Unheck the menu item of the old mode
  RCLCPP_DEBUG(get_logger(), "Old mode item: %d", item_last_);
  if (item_last_) {
    // Assuming that it's never 0, except the very first transition from NONE
    menu_handler_->setCheckState(item_last_,
                                 interactive_markers::MenuHandler::UNCHECKED);
  }

  // Check the menu item of the new mode
  RCLCPP_DEBUG(get_logger(), "New mode item: %d", item_new);
  RCLCPP_DEBUG(get_logger(), "New mode: %d", mode_new);
  menu_handler_->setCheckState(item_new,
                               interactive_markers::MenuHandler::CHECKED);

  // Perform the transition
  modes_[mode_last_]->leave(modes_[mode_new]);
  modes_[mode_new]->enter(modes_[mode_last_]);
  item_last_ = item_new;
  mode_last_ = mode_new;

  menu_handler_->reApply(*server_);
  server_->applyChanges();
  RCLCPP_DEBUG(get_logger(), "modeCb_() done");
}

}  // namespace openvmp_control_interactive
