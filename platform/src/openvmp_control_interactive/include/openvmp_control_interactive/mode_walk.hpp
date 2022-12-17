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
      : TrajVelControl(node, server, WALK, "Walk"), phase_{0}, lift_{0.0} {}
  virtual bool is_whole_body() const override { return true; }

  virtual void enter(std::shared_ptr<ControlImpl> from) override;
  virtual void leave(std::shared_ptr<ControlImpl> to) override;

 private:
  int phase_;
  trajectory_msgs::msg::JointTrajectory msg_template_;
  double lift_;
  static constexpr double LIFT_LIMIT_BOTTOM = -1.0;
  static constexpr double LIFT_LIMIT_TOP = 0.0;

  void next_phase_(int dir, trajectory_msgs::msg::JointTrajectoryPoint &point,
                   const builtin_interfaces::msg::Duration &time_from_start);
  void next_phase_animation_(
      int dir, trajectory_msgs::msg::JointTrajectory &msg,
      const builtin_interfaces::msg::Duration &time_from_start);

  void processFeedback_(
      const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr
          &feedback);
};

}  // namespace openvmp_control_interactive

#endif  // OPENVMP_CONTROL_INTERACTIVE_MODE_WALK_H
