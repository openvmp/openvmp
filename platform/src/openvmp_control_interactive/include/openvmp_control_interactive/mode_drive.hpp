/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-12-12
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_CONTROL_INTERACTIVE_MODE_DRIVE_H
#define OPENVMP_CONTROL_INTERACTIVE_MODE_DRIVE_H

#include <memory>
#include <string>

#include "openvmp_control_interactive/control_traj_vel.hpp"
#include "openvmp_control_interactive/mode.hpp"

namespace openvmp_control_interactive {

class DriveMode : public TrajVelControl {
 public:
  DriveMode(
      rclcpp::Node *node,
      std::shared_ptr<interactive_markers::InteractiveMarkerServer> server)
      : TrajVelControl(node, server, DRIVE, "Drive"), lift_{0.0}, turn_{0.0} {}
  virtual bool is_whole_body() const override { return true; }

  virtual void enter(std::shared_ptr<ControlImpl> from) override;
  virtual void leave(std::shared_ptr<ControlImpl> to) override;

 private:
  trajectory_msgs::msg::JointTrajectory msg_template_;
  trajectory_msgs::msg::JointTrajectoryPoint point_template_;
  double lift_;
  static constexpr double LIFT_LIMIT_BOTTOM = -0.25;
  static constexpr double LIFT_LIMIT_TOP = 0.25;
  double turn_;
  static constexpr double TURN_LIMIT_BOTTOM = -0.6;
  static constexpr double TURN_LIMIT_TOP = 0.6;

  void processFeedback_(
      const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr
          &feedback);
};

}  // namespace openvmp_control_interactive

#endif  // OPENVMP_CONTROL_INTERACTIVE_MODE_DRIVE_H
