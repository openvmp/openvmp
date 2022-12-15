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

#include "builtin_interfaces/msg/duration.hpp"
#include "openvmp_control_interactive/mode_drive.hpp"
#include "openvmp_control_interactive/mode_full.hpp"
#include "openvmp_control_interactive/mode_grab.hpp"
#include "openvmp_control_interactive/mode_hang.hpp"
#include "openvmp_control_interactive/mode_hug.hpp"
#include "openvmp_control_interactive/mode_none.hpp"
#include "openvmp_control_interactive/mode_walk.hpp"
#include "tf2/utils.h"

namespace openvmp_control_interactive {

Node::Node()
    : rclcpp::Node::Node("openvmp_control_interactive"),
      server_(std::make_unique<interactive_markers::InteractiveMarkerServer>(
          "twist_server", get_node_base_interface(), get_node_clock_interface(),
          get_node_logging_interface(), get_node_topics_interface(),
          get_node_services_interface())),
      item_last_{0},
      mode_last_{NONE} {
  RCLCPP_INFO(this->get_logger(), "Namespace: %s",
              this->get_effective_namespace().c_str());

  // Initialize the list of modes
  modes_.add(std::shared_ptr<ControlImpl>(new NoneMode(this, server_)));
  modes_.add(std::shared_ptr<ControlImpl>(new FullMode(this, server_)));
  modes_.add(std::shared_ptr<ControlImpl>(new WalkMode(this, server_)));
  modes_.add(std::shared_ptr<ControlImpl>(new DriveMode(this, server_)));
  modes_.add(std::shared_ptr<ControlImpl>(new HugMode(this, server_)));
  modes_.add(std::shared_ptr<ControlImpl>(new HangMode(this, server_)));
  modes_.add(std::shared_ptr<ControlImpl>(new GrabMode(this, server_)));
  initMenu_();

  // Create menu
  server_->applyChanges();

  menu_modes_[item_last_]->enter(modes_[NONE]);
}

}  // namespace openvmp_control_interactive
