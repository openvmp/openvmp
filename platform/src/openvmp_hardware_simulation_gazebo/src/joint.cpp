/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-09-24
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "openvmp_hardware_simulation_gazebo/joint.hpp"

#include "openvmp_hardware_configuration/gearbox.hpp"

namespace openvmp_hardware_simulation_gazebo {

Joint::Joint(rclcpp::Node *node, gazebo::physics::JointPtr joint,
             std::shared_ptr<openvmp_hardware_configuration::Joint> config)
    : node_{node}, joint_{joint}, config_{config} {}

void Joint::init(std::shared_ptr<Joint> shared_this) {
  auto brake_config = config_->get_brake();
  if (brake_config) {
    brake_ = std::make_shared<Brake>(node_, shared_this, brake_config);
  }
  RCLCPP_DEBUG(node_->get_logger(), "Successfuly initialized the brake");
  auto actuator_config = config_->get_actuator();
  if (actuator_config) {
    actuator_ = std::make_shared<Actuator>(node_, shared_this, actuator_config);
  }

  update();

  RCLCPP_DEBUG(node_->get_logger(), "Successfuly initialized the joint '%s'",
               config_->get_name().c_str());
}

void Joint::update() {
  double friction = 0.0;

  RCLCPP_DEBUG(node_->get_logger(), "Updating the properties of the joint '%s'",
               config_->get_name().c_str());

  if (brake_) {
    friction += brake_->get_friction();
  }
  if (actuator_) {
    friction += actuator_->get_friction();
  }

  auto gearbox_config = config_->get_gearbox();
  if (gearbox_config) {
    friction *= gearbox_config->get_ratio();
    if (gearbox_config->get_type() ==
        openvmp_hardware_configuration::Gearbox::WORM) {
      friction *= openvmp_hardware_configuration::Gearbox::WORM_SELF_LOCK_RATIO;
    }
  }

  // joint_->SetDamping(0, 0.05);
  joint_->SetParam("friction", 0, friction);
}

}  // namespace openvmp_hardware_simulation_gazebo
