/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-10-30
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_HARDWARE_SIMULATION_GAZEBO_PLUGIN_H
#define OPENVMP_HARDWARE_SIMULATION_GAZEBO_PLUGIN_H

#include <memory>
#include <thread>

#include "gazebo/common/Event.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/Model.hh"
#include "gazebo/physics/World.hh"
#include "openvmp_hardware_configuration/configuration.hpp"
#include "openvmp_hardware_simulation_gazebo/joint.hpp"
#include "openvmp_hardware_simulation_gazebo/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sdf/sdf.hh"

namespace openvmp_hardware_simulation_gazebo {

class OpenVMPSimulationPlugin : public gazebo::ModelPlugin {
 public:
  OpenVMPSimulationPlugin();
  virtual ~OpenVMPSimulationPlugin();

  void Load(gazebo::physics::ModelPtr model,
            sdf::ElementPtr plugin_config) override;

 private:
  rclcpp::executors::MultiThreadedExecutor exec_;
  std::shared_ptr<Node> node_;
  std::shared_ptr<std::thread> node_spinner_;

  std::string namespace_;
  std::shared_ptr<openvmp_hardware_configuration::Configuration> configuration_;

  std::map<std::string, std::shared_ptr<Joint>> joints_;
};

}  // namespace openvmp_hardware_simulation_gazebo

#endif  // OPENVMP_HARDWARE_SIMULATION_GAZEBO_PLUGIN_H