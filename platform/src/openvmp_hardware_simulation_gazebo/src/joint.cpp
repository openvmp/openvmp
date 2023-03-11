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
#include "openvmp_hardware_simulation_gazebo/device_node.hpp"
#include "openvmp_hardware_simulation_gazebo/plugin.hpp"

namespace openvmp_hardware_simulation_gazebo {

Joint::Joint(rclcpp::Node *node, gazebo::physics::JointPtr joint,
             std::shared_ptr<openvmp_hardware_configuration::Joint> config)
    : node_{node}, joint_{joint}, config_{config} {}

void Joint::init(OpenVMPSimulationPlugin *plugin,
                 std::shared_ptr<Joint> shared_this) {
  auto brake_config = config_->get_brake();
  if (brake_config) {
    brake_ = std::make_shared<
        DeviceNode<Brake, openvmp_hardware_configuration::Brake> >(
        node_->get_namespace(),
        "simulation_driver_joint_" + config_->get_name() + "_brake",
        shared_this, brake_config,
        std::vector<rclcpp::Parameter>{
            rclcpp::Parameter("brake_prefix", brake_config->get_prefix()),
        });
    plugin->addSubNode(brake_);
  }
  RCLCPP_DEBUG(node_->get_logger(), "Successfuly initialized the brake");

  auto actuator_config = config_->get_actuator();
  if (actuator_config) {
    actuator_ = std::make_shared<
        DeviceNode<Actuator, openvmp_hardware_configuration::Actuator> >(
        node_->get_namespace(),
        "simulation_driver_joint_" + config_->get_name() + "_actuator",
        shared_this, actuator_config,
        std::vector<rclcpp::Parameter>{
            rclcpp::Parameter("actuator_prefix", actuator_config->get_prefix()),
        });
    plugin->addSubNode(actuator_);
  }
  RCLCPP_DEBUG(node_->get_logger(), "Successfuly initialized the actuator");

  auto encoder_config = config_->get_encoder();
  if (encoder_config) {
    encoder_ = std::make_shared<
        DeviceNode<Encoder, openvmp_hardware_configuration::Encoder> >(
        node_->get_namespace(),
        "simulation_driver_joint_" + config_->get_name() + "_encoder",
        shared_this, encoder_config,
        std::vector<rclcpp::Parameter>{
            rclcpp::Parameter("encoder_prefix", encoder_config->get_prefix()),
            rclcpp::Parameter("encoder_readings_per_second", 50.0),
        });
    plugin->addSubNode(encoder_);
  }
  RCLCPP_DEBUG(node_->get_logger(), "Successfuly initialized the actuator");

  updateFriction();

  RCLCPP_DEBUG(node_->get_logger(), "Successfuly initialized the joint '%s'",
               config_->get_name().c_str());
}

void Joint::updateFriction() {
  double friction = 0.0;

  RCLCPP_DEBUG(node_->get_logger(), "Updating the properties of the joint '%s'",
               config_->get_name().c_str());

  if (brake_) {
    friction += brake_->device->get_friction();
  }
  if (actuator_) {
    friction += actuator_->device->get_friction();
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

void Joint::setPosition(double position) {
  joint_->SetPosition(0, position /* * 2.0 * M_PI*/);
}

void Joint::setVelocity(double velocity) {
  // TODO(clairbee): consider simulating auto-release of brakes
  // if (brake_) {
  //   brake_->device->set_engaged(velocity > 0.00001);
  // }
  // updateFriction();
  joint_->SetParam("fmax", 0, 2000.0);
  joint_->SetParam("vel", 0, velocity * 2.0 * M_PI);
}

double Joint::getPosition() {
  return joint_->Position(0) /* / / (2.0 * M_PI) */;
}

double Joint::getVelocity() { return joint_->GetVelocity(0) / (2.0 * M_PI); }

}  // namespace openvmp_hardware_simulation_gazebo
