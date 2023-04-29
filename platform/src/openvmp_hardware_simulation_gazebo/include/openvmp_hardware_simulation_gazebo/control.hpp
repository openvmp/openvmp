/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-04-29
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_HARDWARE_SIMULATION_GAZEBO_CONTROL_H
#define OPENVMP_HARDWARE_SIMULATION_GAZEBO_CONTROL_H

#include <memory>
#include <string>

#include "controller_interface/controller_interface.hpp"
#include "controller_manager/controller_manager.hpp"
#include "gazebo/common/Event.hh"
#include "gazebo/common/Events.hh"
#include "gazebo/physics/Model.hh"
#include "gazebo/physics/PhysicsEngine.hh"
#include "gazebo/physics/World.hh"
#include "hardware_interface/component_parser.hpp"
#include "hardware_interface/resource_manager.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "remote_hardware_interface/system_interface.hpp"

namespace openvmp_hardware_simulation_gazebo {

class Control {
 public:
  Control(rclcpp::Node *node,
          std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> exec,
          gazebo::physics::ModelPtr model);
  virtual ~Control();

 private:
  rclcpp::Node *node_;
  gazebo::physics::ModelPtr model_;

  // Pointer to the update event connection
  gazebo::event::ConnectionPtr update_connection_;

  // Interface loader
  boost::shared_ptr<
      pluginlib::ClassLoader<remote_hardware_interface::RemoteSystemInterface>>
      robot_hw_sim_loader_;

  // Controller manager
  std::shared_ptr<controller_manager::ControllerManager> controller_manager_;

  // Available controllers
  std::vector<std::shared_ptr<controller_interface::ControllerInterface>>
      controllers_;

  // Timing
  rclcpp::Duration control_period_ = rclcpp::Duration(1, 0);

  // Last time the update method was called
  rclcpp::Time last_update_sim_time_ros_ =
      rclcpp::Time((int64_t)0, RCL_ROS_TIME);

  std::string get_urdf_() const;
  void update_();
  void reset_();
};

}  // namespace openvmp_hardware_simulation_gazebo

#endif  // OPENVMP_HARDWARE_SIMULATION_GAZEBO_CONTROL_H
