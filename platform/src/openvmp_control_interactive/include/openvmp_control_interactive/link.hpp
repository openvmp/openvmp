/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-10-25
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_CONTROL_INTERACTIVE_LINK_H
#define OPENVMP_CONTROL_INTERACTIVE_LINK_H

#include <map>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

namespace openvmp_control_interactive {

struct Link {
  int index;
  enum axis { X, Y, Z };

  const axis axis_vis, axis_rot;
  const uint8_t mode;

  double marker_size_scale = 1;

  double linear_drive_scale = 1, angular_drive_scale = 1,
         max_linear_velocity = 1, max_angular_velocity = 1;
};

extern const int LINKS_TOTAL;
extern const std::map<const std::string, Link> &get_links();

}  // namespace openvmp_control_interactive

#endif  // OPENVMP_CONTROL_INTERACTIVE_LINK_H
