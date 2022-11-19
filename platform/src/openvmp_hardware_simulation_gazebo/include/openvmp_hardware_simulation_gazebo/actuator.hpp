/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-11-12
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_HARDWARE_SIMULATION_GAZEBO_ACTUATOR_H
#define OPENVMP_HARDWARE_SIMULATION_GAZEBO_ACTUATOR_H

#include <memory>
#include <string>

/* not yet
#include "actuator/interface.hpp"
#include "actuator/srv/command.hpp"
*/
#include "gazebo/physics/Joint.hh"
#include "openvmp_hardware_configuration/actuator.hpp"
#include "rclcpp/rclcpp.hpp"

namespace openvmp_hardware_simulation_gazebo {

class Joint;

class Actuator /*: public actuator::Interface*/ {
 public:
  Actuator(rclcpp::Node *node, std::weak_ptr<Joint> joint,
           std::shared_ptr<openvmp_hardware_configuration::Actuator> config);
  virtual ~Actuator() {}

  double get_friction();

 protected:
  /*
   virtual void command_handler_real_(
       const std::shared_ptr<actuator::srv::Command::Request> request,
       std::shared_ptr<actuator::srv::Command::Response> response) override;
       */

  std::weak_ptr<Joint> joint_;

  enum Mode { DETENT, STALLING, RUNNING };

 private:
  Mode mode_ = DETENT;
  double torque_;
  double torque_detent_;
  double torque_stalling_;
};

}  // namespace openvmp_hardware_simulation_gazebo

#endif  // OPENVMP_HARDWARE_SIMULATION_GAZEBO_ACTUATOR_H
