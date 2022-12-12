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

// TODO(clairbee): move all of these link definitions into "move_full.cpp"
// TODO(clairbee): consider implementing all modes via links thus reducing code

const auto robot_arm = [](const std::string &side, const std::string &arm,
                          int offset, bool half_invert, bool side_invert) {
  return std::map<const std::string, Link>({
      {side + "_" + arm + "_arm_lower_part",
       Link{
           .index = offset,
           .joint = side + "_" + arm + "_arm_joint",
           .axis_vis = Link::Z,
           .axis_rot = Link::Y,
           .mode =
               visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS,
           .marker_size_scale = 0.2,
           .invert = half_invert,
       }},
      {side + "_" + arm + "_arm_upper_part",
       Link{
           .index = offset + 1,
           .joint = side + "_" + arm + "_arm_inner_joint",
           .axis_vis = Link::X,
           .axis_rot = Link::X,
           .mode =
               visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS,
           .marker_size_scale = 0.2,
           .invert = side_invert,
       }},
      // {side + "_" + arm + "_left_arm_wheel",
      //  Link{
      //      .index = offset + 2,
      //      .axis_vis = Link::X,
      //      .axis_rot = Link::X,
      //      .mode =
      //          visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS,
      //      .marker_size_scale = 0.2,
      //  }},
  });
};

const auto robot_half = [](const std::string &side, int offset, bool invert) {
  std::map<const std::string, Link> half({
      {side + "_turn_table_link",
       Link{
           .index = offset,
           .joint = side + "_turn_table_joint",
           .axis_vis = Link::Y,
           .axis_rot = Link::Z,
           .mode =
               visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS,
           .marker_size_scale = 0.25,
           .invert = true,
       }},
      {side + "_body_link",
       Link{
           .index = offset + 1,
           .joint = side + "_body_joint",
           .axis_vis = Link::X,
           .axis_rot = Link::X,
           .mode =
               visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS,
           .marker_size_scale = 0.3,
           .invert = !invert,
       }},
  });
  const auto &left = robot_arm(side, "left", offset + 2, invert, true);
  const auto &right = robot_arm(side, "right", offset + 4, invert, false);
  half.insert(left.begin(), left.end());
  half.insert(right.begin(), right.end());
  return half;
};

const int LINKS_TOTAL = 12;  // This must agree with ros2_controllers.yaml
static std::map<const std::string, Link> links;
static std::mutex links_lock;
static bool links_initialized;

std::map<const std::string, Link> &get_links() {
  std::lock_guard<std::mutex> lockGuard(links_lock);

  if (!links_initialized) {
    const auto &front = robot_half("front", 0, true);
    const auto &rear = robot_half("rear", LINKS_TOTAL / 2, false);
    links.insert(front.begin(), front.end());
    links.insert(rear.begin(), rear.end());
    links_initialized = true;
  }
  return links;
}

}  // namespace openvmp_control_interactive
