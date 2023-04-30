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

  position_did_set_(position);
}

void Actuator::velocity_set_real_(double velocity) {
  if (std::fabs(velocity) < 0.00001) {
    mode_ = DETENT;
  } else {
    mode_ = RUNNING;
  }

  auto joint_ptr = joint_.lock();
  joint_ptr->setVelocity(velocity);

  velocity_did_set_(velocity);
}

}  // namespace openvmp_hardware_simulation_gazebo
