/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-11
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_HARDWARE_SIMULATION_GAZEBO_ENCODER_H
#define OPENVMP_HARDWARE_SIMULATION_GAZEBO_ENCODER_H

#include <memory>
#include <string>

#include "gazebo/physics/Joint.hh"
#include "openvmp_hardware_configuration/encoder.hpp"
#include "rclcpp/rclcpp.hpp"
#include "remote_encoder/implementation.hpp"

namespace openvmp_hardware_simulation_gazebo {

class Joint;

class Encoder : public remote_encoder::Implementation {
 public:
  Encoder(rclcpp::Node *node, std::weak_ptr<Joint> joint,
          std::shared_ptr<openvmp_hardware_configuration::Encoder> config);
  virtual ~Encoder() {}

 protected:
  virtual bool has_position() override { return true; }
  virtual bool has_velocity() override { return true; }
  virtual void position_get_real_() override;
  virtual void velocity_get_real_() override;

  std::weak_ptr<Joint> joint_;

 private:
  // double type_;
};

}  // namespace openvmp_hardware_simulation_gazebo

#endif  // OPENVMP_HARDWARE_SIMULATION_GAZEBO_ENCODER_H
