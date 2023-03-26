/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-11-12
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_HARDWARE_SIMULATION_GAZEBO_BREAK_H
#define OPENVMP_HARDWARE_SIMULATION_GAZEBO_BREAK_H

#include <memory>
#include <string>

#include "gazebo/physics/Joint.hh"
#include "openvmp_hardware_configuration/brake.hpp"
#include "rclcpp/rclcpp.hpp"
#include "remote_brake/interface.hpp"
#include "remote_brake/srv/command.hpp"

namespace openvmp_hardware_simulation_gazebo {

class Joint;

class Brake : public remote_brake::Interface {
 public:
  Brake(rclcpp::Node *node, std::weak_ptr<Joint> joint,
        std::shared_ptr<openvmp_hardware_configuration::Brake> config);
  virtual ~Brake() {}

  double get_friction();

 protected:
  virtual void command_handler_real_(
      const std::shared_ptr<remote_brake::srv::Command::Request> request,
      std::shared_ptr<remote_brake::srv::Command::Response> response) override;

  std::weak_ptr<Joint> joint_;

 private:
  double torque_;
};

}  // namespace openvmp_hardware_simulation_gazebo

#endif  // OPENVMP_HARDWARE_SIMULATION_GAZEBO_BREAK_H
