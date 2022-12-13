/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-12-10
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_CONTROL_INTERACTIVE_CONTROL_TRAJ_VEL_H
#define OPENVMP_CONTROL_INTERACTIVE_CONTROL_TRAJ_VEL_H

#include <mutex>
#include <string>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "openvmp_control_interactive/control.hpp"
#include "openvmp_control_interactive/link.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace openvmp_control_interactive {

class TrajVelControl : public ControlImpl {
 public:
  TrajVelControl(
      rclcpp::Node *node,
      std::shared_ptr<interactive_markers::InteractiveMarkerServer> server,
      Mode mode, const std::string &name)
      : ControlImpl(node, server, mode, name) {}
  virtual ~TrajVelControl();

 protected:
  static rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr
      trajectory_commands_;
  // rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
  //     position_commands_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
      velocity_commands_;

  virtual Control get_control() const override { return TRAJ_VEL; }
  virtual void init() override;
  virtual void fini() override;

 private:
  static std::mutex lock_;
  static bool initialized_;
};

}  // namespace openvmp_control_interactive

#endif  // OPENVMP_CONTROL_INTERACTIVE_CONTROL_TRAJ_VEL_H
