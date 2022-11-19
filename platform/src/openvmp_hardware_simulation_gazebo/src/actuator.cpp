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
    : /*brake::Interface(node),*/ joint_{joint} {
  (void)node;

  torque_ = config->get_torque();
  torque_detent_ = config->get_torque_detent();
  torque_stalling_ = config->get_torque_stalling();
}

/* not yet
void Actuator::command_handler_real_(
    const std::shared_ptr<actuator::srv::Command::Request> request,
    std::shared_ptr<actuator::srv::Command::Response> response) {
  ... = request->...;

  // Update the joint friction parameters
  auto joint_ptr = joint_.lock();
  joint_ptr->update();

  response->exception_code = 0;
}
*/

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

}  // namespace openvmp_hardware_simulation_gazebo