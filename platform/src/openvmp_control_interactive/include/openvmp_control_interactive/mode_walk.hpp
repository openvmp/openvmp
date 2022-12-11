/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-12-10
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_CONTROL_INTERACTIVE_MODE_WALK_H
#define OPENVMP_CONTROL_INTERACTIVE_MODE_WALK_H

#include <memory>
#include <string>

#include "openvmp_control_interactive/control_traj_vel.hpp"
#include "openvmp_control_interactive/mode.hpp"

namespace openvmp_control_interactive {

class WalkMode : public TrajVelControl {
 public:
  WalkMode(rclcpp::Node *node,
           std::shared_ptr<interactive_markers::InteractiveMarkerServer> server)
      : TrajVelControl(node, server, WALK, "Walk") {}

  virtual void enter(std::shared_ptr<ControlImpl> from) override;
  virtual void leave(std::shared_ptr<ControlImpl> to) override;

 private:
  void processFeedback_(
      const std::string &marker_name,
      const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr
          &feedback);
};

}  // namespace openvmp_control_interactive

#endif  // OPENVMP_CONTROL_INTERACTIVE_MODE_WALK_H
