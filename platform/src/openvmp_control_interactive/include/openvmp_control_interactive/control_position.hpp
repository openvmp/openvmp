/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-12-17
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_CONTROL_INTERACTIVE_CONTROL_POSITION_H
#define OPENVMP_CONTROL_INTERACTIVE_CONTROL_POSITION_H

#include <mutex>
#include <string>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "openvmp_control_interactive/control.hpp"
#include "openvmp_control_interactive/link.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace openvmp_control_interactive {

class PositionControl : public ControlImpl {
 public:
  PositionControl(
      rclcpp::Node *node,
      std::shared_ptr<interactive_markers::InteractiveMarkerServer> server,
      Mode mode, const std::string &name)
      : ControlImpl(node, server, mode, name) {}
  virtual ~PositionControl();

 protected:
  static rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
      position_commands_;

  virtual Control get_control() const override { return POSITION; }
  virtual void init() override;
  virtual void fini() override;

 private:
  static std::mutex lock_;
  static bool initialized_;
};

}  // namespace openvmp_control_interactive

#endif  // OPENVMP_CONTROL_INTERACTIVE_CONTROL_POSITION_H
