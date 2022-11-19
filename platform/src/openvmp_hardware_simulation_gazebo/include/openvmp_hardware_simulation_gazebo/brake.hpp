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

#include "brake/interface.hpp"
#include "brake/srv/command.hpp"
#include "gazebo/physics/Joint.hh"
#include "openvmp_hardware_configuration/brake.hpp"
#include "rclcpp/rclcpp.hpp"

namespace openvmp_hardware_simulation_gazebo {

class Joint;

class Brake : public brake::Interface {
 public:
  Brake(rclcpp::Node *node, std::weak_ptr<Joint> joint,
        std::shared_ptr<openvmp_hardware_configuration::Brake> config);
  virtual ~Brake() {}

  double get_friction();

 protected:
  virtual void command_handler_real_(
      const std::shared_ptr<brake::srv::Command::Request> request,
      std::shared_ptr<brake::srv::Command::Response> response) override;

  std::weak_ptr<Joint> joint_;

 private:
  double torque_;
};

}  // namespace openvmp_hardware_simulation_gazebo

#endif  // OPENVMP_HARDWARE_SIMULATION_GAZEBO_BREAK_H
