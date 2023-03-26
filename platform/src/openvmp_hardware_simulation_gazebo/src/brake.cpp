/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-09-24
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "openvmp_hardware_simulation_gazebo/brake.hpp"

#include "openvmp_hardware_simulation_gazebo/joint.hpp"

namespace openvmp_hardware_simulation_gazebo {

Brake::Brake(rclcpp::Node *node, std::weak_ptr<Joint> joint,
             std::shared_ptr<openvmp_hardware_configuration::Brake> config)
    : remote_brake::Interface(node, config->get_engaged_by_default()),
      joint_{joint} {
  torque_ = config->get_torque();
}

void Brake::command_handler_real_(
    const std::shared_ptr<remote_brake::srv::Command::Request> request,
    std::shared_ptr<remote_brake::srv::Command::Response> response) {
  // Update the joint friction parameters
  auto joint_ptr = joint_.lock();
  joint_ptr->updateFriction();

  response->exception_code = 0;
}

double Brake::get_friction() {
  if (engaged_) {
    return torque_;
  } else {
    return 0.0;
  }
}

}  // namespace openvmp_hardware_simulation_gazebo