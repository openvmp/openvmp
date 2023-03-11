/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-11
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "openvmp_hardware_simulation_gazebo/encoder.hpp"

#include "openvmp_hardware_simulation_gazebo/joint.hpp"

namespace openvmp_hardware_simulation_gazebo {

Encoder::Encoder(
    rclcpp::Node *node, std::weak_ptr<Joint> joint,
    std::shared_ptr<openvmp_hardware_configuration::Encoder> config)
    : remote_encoder::Implementation(node), joint_{joint} {
  (void)node;

  // type_ = config->get_type();

  init_encoder();
}

void Encoder::position_get_real_() {
  auto joint_ptr = joint_.lock();
  position_last_ = joint_ptr->getPosition();
}

void Encoder::velocity_get_real_() {
  auto joint_ptr = joint_.lock();
  velocity_last_ = joint_ptr->getVelocity();
}

}  // namespace openvmp_hardware_simulation_gazebo