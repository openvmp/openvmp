/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-10-30
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "openvmp_gazebo_hardware_simulation/plugin.hpp"

#include "sdf/sdf.hh"

namespace openvmp_gazebo_hardware_simulation {

OpenVMPSimulationPlugin::OpenVMPSimulationPlugin()
    : gazebo::ModelPlugin(),
      logger_{rclcpp::get_logger("openvmp_gazebo_hardware_simulation")} {}

void OpenVMPSimulationPlugin::Load(gazebo::physics::ModelPtr model,
                                   sdf::ElementPtr sdf) {
  RCLCPP_ERROR(logger_, "Initializing...");

  sdf::ElementPtr joint = sdf->FindElement("joint");
  if (!joint) {
    RCLCPP_ERROR(logger_, "No joints found!");
  } else {
    do {
      auto name = joint->Get<std::string>("name");
      RCLCPP_ERROR(logger_, "Found joint %s", name.c_str());

      sdf::ElementPtr next;

      next = joint->FindElement("gearbox");
      if (next) {
        do {
          RCLCPP_ERROR(logger_, "Found a gearbox!");
        } while ((next = next->GetNextElement("gearbox")) != nullptr);
      }

      next = joint->FindElement("motor");
      if (next) {
        do {
          RCLCPP_ERROR(logger_, "Found a motor!");
        } while ((next = next->GetNextElement("motor")) != nullptr);
      }

      next = joint->FindElement("brake");
      if (next) {
        do {
          RCLCPP_ERROR(logger_, "Found a brake!");
        } while ((next = joint->GetNextElement("brake")) != nullptr);
      }

      next = joint->FindElement("sensor");
      if (next) {
        do {
          RCLCPP_ERROR(logger_, "Found a sensor!");
        } while ((next = joint->GetNextElement("sensor")) != nullptr);
      }
    } while ((joint = joint->GetNextElement("joint")) != nullptr);
  }
}

GZ_REGISTER_MODEL_PLUGIN(OpenVMPSimulationPlugin)
}  // namespace openvmp_gazebo_hardware_simulation
