/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-11-12
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_HARDWARE_SIMULATION_GAZEBO_JOINT_H
#define OPENVMP_HARDWARE_SIMULATION_GAZEBO_JOINT_H

#include <memory>
#include <string>

#include "gazebo/physics/Joint.hh"
#include "openvmp_hardware_configuration/configuration.hpp"
#include "openvmp_hardware_simulation_gazebo/actuator.hpp"
#include "openvmp_hardware_simulation_gazebo/brake.hpp"
#include "openvmp_hardware_simulation_gazebo/device_node.hpp"
#include "openvmp_hardware_simulation_gazebo/encoder.hpp"
#include "rclcpp/rclcpp.hpp"

namespace openvmp_hardware_simulation_gazebo {

class OpenVMPSimulationPlugin;

class Joint {
 public:
  Joint(rclcpp::Node *node, gazebo::physics::JointPtr joint,
        std::shared_ptr<openvmp_hardware_configuration::Joint> config);
  virtual ~Joint() {}

  // init is called to initialize all properties that require access to the
  // shared pointer to this object itself
  void init(OpenVMPSimulationPlugin *plugin,
            std::shared_ptr<Joint> shared_this);

  // updateFriction is called by brakes and actuators to calculate the combined
  // friction
  void updateFriction();

  // setPosition sets the joint angle
  void setPosition(double velocity);
  // getPosition gets the joint angle
  double getPosition();
  // setVelocity sets the joint velocity
  void setVelocity(double velocity);
  // getVelocity gets the joint velocity
  double getVelocity();

 private:
  rclcpp::Node *node_;
  gazebo::physics::JointPtr joint_;
  std::shared_ptr<openvmp_hardware_configuration::Joint> config_;
  bool initialized_ = false;

  std::shared_ptr<
      DeviceNode<Actuator, openvmp_hardware_configuration::Actuator>>
      actuator_;
  std::shared_ptr<DeviceNode<Encoder, openvmp_hardware_configuration::Encoder>>
      encoder_;
  std::shared_ptr<DeviceNode<Brake, openvmp_hardware_configuration::Brake>>
      brake_;
};

}  // namespace openvmp_hardware_simulation_gazebo

#endif  // OPENVMP_HARDWARE_SIMULATION_GAZEBO_JOINT_H
