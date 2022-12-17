/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-12-10
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_CONTROL_INTERACTIVE_MODE_FULL_H
#define OPENVMP_CONTROL_INTERACTIVE_MODE_FULL_H

#include <memory>
#include <string>

#include "control_msgs/msg/joint_trajectory_controller_state.hpp"
#include "openvmp_control_interactive/control_traj_vel.hpp"
#include "openvmp_control_interactive/link.hpp"
#include "openvmp_control_interactive/mode.hpp"

namespace openvmp_control_interactive {

class FullMode : public TrajVelControl {
 public:
  FullMode(
      rclcpp::Node *node,
      std::shared_ptr<interactive_markers::InteractiveMarkerServer> server);
  virtual ~FullMode();
  virtual bool is_whole_body() const override { return true; }

  virtual void enter(std::shared_ptr<ControlImpl> from) override;
  virtual void leave(std::shared_ptr<ControlImpl> to) override;

 private:
  std::mutex state_lock_;
  rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::
      SharedPtr trajectory_state_subscription_;

  void trajectoryStateHandler_(
      const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg);
  void processFeedback_(
      const std::string &marker_name, const std::string &link_name,
      const Link &link,
      const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr
          &feedback);
};

}  // namespace openvmp_control_interactive

#endif  // OPENVMP_CONTROL_INTERACTIVE_MODE_FULL_H
