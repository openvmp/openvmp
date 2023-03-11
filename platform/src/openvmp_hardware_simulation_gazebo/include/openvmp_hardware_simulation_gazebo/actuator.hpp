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

#include "gazebo/physics/Joint.hh"
#include "openvmp_hardware_configuration/actuator.hpp"
#include "rclcpp/rclcpp.hpp"
#include "remote_actuator/implementation.hpp"

namespace openvmp_hardware_simulation_gazebo {

class Joint;

class Actuator : public remote_actuator::Implementation {
 public:
  Actuator(rclcpp::Node *node, std::weak_ptr<Joint> joint,
           std::shared_ptr<openvmp_hardware_configuration::Actuator> config);
  virtual ~Actuator() {}

  double get_friction();

 protected:
  virtual bool has_position() override { return true; }
  virtual bool has_velocity() override { return true; }
  virtual void position_set_real_(double) override;
  virtual void velocity_set_real_(double) override;

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
