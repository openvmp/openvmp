/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-10-30
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_GAZEBO_HARDWARE_SIMULATION_PLUGIN_H
#define OPENVMP_GAZEBO_HARDWARE_SIMULATION_PLUGIN_H

#include <memory>

#include "gazebo/common/Event.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/Model.hh"
#include "gazebo/physics/World.hh"
#include "openvmp_gazebo_hardware_simulation/brakes.hpp"
#include "rclcpp/rclcpp.hpp"

namespace openvmp_gazebo_hardware_simulation {

class OpenVMPSimulationPlugin : public gazebo::ModelPlugin {
 public:
  OpenVMPSimulationPlugin();
  virtual ~OpenVMPSimulationPlugin() {}

  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;

 private:
  rclcpp::Logger logger_;
  std::shared_ptr<BrakesSimulation> brakes_;
};

}  // namespace openvmp_gazebo_hardware_simulation

#endif  // OPENVMP_GAZEBO_HARDWARE_SIMULATION_PLUGIN_H