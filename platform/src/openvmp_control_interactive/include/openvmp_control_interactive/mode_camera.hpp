/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-12-17
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_CONTROL_INTERACTIVE_MODE_CAMERAS_H
#define OPENVMP_CONTROL_INTERACTIVE_MODE_CAMERAS_H

#include <map>
#include <memory>
#include <string>

#include "openvmp_control_interactive/control_position.hpp"
#include "openvmp_control_interactive/mode.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace openvmp_control_interactive {

class CamerasMode : public PositionControl {
 public:
  CamerasMode(
      rclcpp::Node *node,
      std::shared_ptr<interactive_markers::InteractiveMarkerServer> server,
      Mode mode);
  virtual bool is_cameras() const override { return true; }

  virtual void enter(std::shared_ptr<ControlImpl> from) override;
  virtual void leave(std::shared_ptr<ControlImpl> to) override;

 private:
  const std::string &prefix_;
  static const std::vector<const std::string> joints_;
  static std::map<const std::string, double> joints_state_;

  void processFeedback_(
      const std::string &link_name,
      const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr
          &feedback);
};

}  // namespace openvmp_control_interactive

#endif  // OPENVMP_CONTROL_INTERACTIVE_MODE_CAMERAS_H
