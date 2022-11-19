/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-10-30
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "openvmp_hardware_simulation_gazebo/plugin.hpp"

#include <chrono>

#include "gazebo/physics/Joint.hh"
#include "gazebo/physics/PhysicsTypes.hh"

namespace openvmp_hardware_simulation_gazebo {

OpenVMPSimulationPlugin::OpenVMPSimulationPlugin() {
  // See "delayed initialization" section below in Load()
}

OpenVMPSimulationPlugin::~OpenVMPSimulationPlugin() {
  if (node_) {
    joints_.clear();

    // All objects dependent on 'node_' must be destroyed before this.
    exec_.cancel();
    node_spinner_->join();
    node_spinner_.reset();
  }
}

void OpenVMPSimulationPlugin::Load(gazebo::physics::ModelPtr model,
                                   sdf::ElementPtr plugin_config) {
  auto logger = rclcpp::get_logger("OpenVMPSimulationPlugin");
  RCLCPP_DEBUG(logger, "Initializing...");

  auto namespace_el = plugin_config->GetElement("ns");
  if (namespace_el) {
    namespace_ = namespace_el->Get<std::string>();
    RCLCPP_INFO(logger, "The namespace parameter is detected: %s",
                namespace_.c_str());
  }

  try {
    // Delayed initialization starts
    node_ = std::make_shared<Node>(namespace_);
    exec_.add_node(node_);
    node_spinner_ = std::make_shared<std::thread>(
        std::bind(&rclcpp::executors::MultiThreadedExecutor::spin, &exec_));
    // Delayed initialization ends
    std::this_thread::sleep_for(std::chrono::milliseconds(
        1000));  // Give the node a chance to initialize
  } catch (const std::exception &e) {
    RCLCPP_ERROR(logger, "Exception while initializing the node: %s.",
                 e.what());
  }

  auto config_el = plugin_config->GetElement("hardware_configuration");
  if (!config_el) {
    RCLCPP_ERROR(node_->get_logger(),
                 "Didn't find 'hardware_configuration' in the model.");
    return;
  }

  const std::string config_filename = config_el->Get<std::string>();
  RCLCPP_DEBUG(node_->get_logger(), "Loading the hardware configuration: %s",
               config_filename.c_str());
  try {
    configuration_ =
        std::make_shared<openvmp_hardware_configuration::Configuration>(
            config_filename);
  } catch (std::exception e) {
    RCLCPP_ERROR(node_->get_logger(),
                 "Exception while parsing 'hardware_configuration': %s",
                 e.what());
  }

  for (auto &joint_config_it : configuration_->get_joints()) {
    auto &joint_config = joint_config_it.second;
    auto name = joint_config->get_name();
    RCLCPP_INFO(node_->get_logger(), "Preparing the joint '%s'", name.c_str());

    try {
      auto model_joint = model->GetJoint(name);
      if (!model_joint) {
        RCLCPP_ERROR(node_->get_logger(),
                     "Didn't find the joint '%s' in the Gazebo model.",
                     name.c_str());
        continue;
      }
      RCLCPP_DEBUG(node_->get_logger(), "Found the joint.");

      // TODO(clairbee): figure out what to do with 'damping'
      // model_joint->SetDamping(0, 0.9);
      // model_joint->SetParam("damping", 0, 0.9);

      auto joint =
          std::make_shared<Joint>(node_.get(), model_joint, joint_config);
      joint->init(joint);
      joints_.insert({name, joint});
    } catch (const std::exception &e) {
      RCLCPP_ERROR(node_->get_logger(),
                   "Exception while preparing the joint: %s", e.what());
    }
  }
}

GZ_REGISTER_MODEL_PLUGIN(OpenVMPSimulationPlugin)

}  // namespace openvmp_hardware_simulation_gazebo
