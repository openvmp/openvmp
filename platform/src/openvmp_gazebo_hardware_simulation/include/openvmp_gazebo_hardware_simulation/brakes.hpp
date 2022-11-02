/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-10-30
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_GAZEBO_HARDWARE_SIMULATION_BRAKES_H
#define OPENVMP_GAZEBO_HARDWARE_SIMULATION_BRAKES_H

#include "gazebo/common/Event.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/Model.hh"
#include "gazebo/physics/World.hh"

namespace openvmp_gazebo_hardware_simulation {

class BrakesSimulation {
 public:
  BrakesSimulation();

  virtual ~BrakesSimulation() {}
};

}  // namespace openvmp_gazebo_hardware_simulation

#endif  // OPENVMP_GAZEBO_HARDWARE_SIMULATION_BRAKES_H