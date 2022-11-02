/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-10-25
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "openvmp_control_interactive/link.hpp"

#include "visualization_msgs/msg/interactive_marker_control.hpp"

namespace openvmp_control_interactive {

const auto robot_half = [](const std::string &side, int offset) {
  return std::map<const std::string, Link>(
      {{side + "_turn_table_link",
        Link{
            .index = offset,
            .axis = Link::Y,
            .mode =
                visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS,
            .marker_size_scale = 0.25,
        }},
       {side + "_body_link",
        Link{
            .index = offset + 1,
            .axis = Link::X,
            .mode =
                visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS,
            .marker_size_scale = 0.3,
        }}});
};

const int LINKS_TOTAL = 4;  // This must agree with ros2_controllers.yaml
static std::map<const std::string, Link> links;
static std::mutex links_lock;
static bool links_initialized;

const std::map<const std::string, Link> &get_links() {
  std::lock_guard<std::mutex> lockGuard(links_lock);

  if (!links_initialized) {
    const auto &front = robot_half("front", 0);
    const auto &rear =
        robot_half("rear", 2);  // This must agree with ros2_controllers.yaml
    links.insert(front.begin(), front.end());
    links.insert(rear.begin(), rear.end());
    links_initialized = true;
  }
  return links;
}

}  // namespace openvmp_control_interactive
