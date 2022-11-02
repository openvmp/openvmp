/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-10-11
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "openvmp_robot_don1/node.hpp"

#include <chrono>
#include <thread>

namespace openvmp_robot_don1 {

Node::Node() : openvmp_robot::Node::Node("don1") {
  std::this_thread::sleep_for(std::chrono::seconds(15));
  preset_motion_walk();
}

}  // namespace openvmp_robot_don1
