/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-12-14
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_CONTROL_INTERACTIVE_MODE_LIFT_H
#define OPENVMP_CONTROL_INTERACTIVE_MODE_LIFT_H

#include <memory>
#include <string>

#include "openvmp_control_interactive/control_traj_vel.hpp"
#include "openvmp_control_interactive/mode.hpp"

namespace openvmp_control_interactive {

class LiftMode : public TrajVelControl {
 public:
  LiftMode(rclcpp::Node *node,
           std::shared_ptr<interactive_markers::InteractiveMarkerServer> server)
      : TrajVelControl(node, server, LIFT, "Lift"), turn_{0.0}, lean_{0.0} {}
  virtual bool is_whole_body() const override { return true; }

  virtual void enter(std::shared_ptr<ControlImpl> from) override;
  virtual void leave(std::shared_ptr<ControlImpl> to) override;

 private:
  trajectory_msgs::msg::JointTrajectory msg_template_;
  trajectory_msgs::msg::JointTrajectoryPoint point_template_;
  double turn_;
  static constexpr double TURN_LIMIT_BOTTOM = -1.0;
  static constexpr double TURN_LIMIT_TOP = 1.0;
  double lean_;
  static constexpr double LEAN_LIMIT_BOTTOM = -0.5;
  static constexpr double LEAN_LIMIT_TOP = 0.5;

  void processFeedback_(
      const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr
          &feedback);
};

}  // namespace openvmp_control_interactive

#endif  // OPENVMP_CONTROL_INTERACTIVE_MODE_LIFT_H
