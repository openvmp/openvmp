/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-09-24
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "openvmp_hardware_simulation_gazebo/actuator.hpp"

#include "openvmp_hardware_simulation_gazebo/joint.hpp"

namespace openvmp_hardware_simulation_gazebo {

Actuator::Actuator(
    rclcpp::Node *node, std::weak_ptr<Joint> joint,
    std::shared_ptr<openvmp_hardware_configuration::Actuator> config)
    : remote_actuator::Implementation(node), joint_{joint} {
  (void)node;

  torque_ = config->get_torque();
  torque_detent_ = config->get_torque_detent();
  torque_stalling_ = config->get_torque_stalling();

  init_actuator();
}

double Actuator::get_friction() {
  switch (mode_) {
    case DETENT:
      return torque_detent_;
    case STALLING:
      return torque_stalling_;
    default:  // case RUNNING:
              // TODO(clairbee): implement torque curves here
      return torque_;
  }
}

void Actuator::position_set_real_(double position) {
  auto joint_ptr = joint_.lock();
  joint_ptr->setPosition(position);
}

void Actuator::velocity_set_real_(double velocity) {
  auto joint_ptr = joint_.lock();
  if (velocity < 0.0001) {
    mode_ = DETENT;
  } else {
    mode_ = RUNNING;
  }
  joint_ptr->setVelocity(velocity);
}

}  // namespace openvmp_hardware_simulation_gazebo