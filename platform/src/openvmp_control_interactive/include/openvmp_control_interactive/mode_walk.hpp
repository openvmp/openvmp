/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-12-10
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_CONTROL_INTERACTIVE_MODE_WALK_H
#define OPENVMP_CONTROL_INTERACTIVE_MODE_WALK_H

#include <map>
#include <memory>
#include <string>

#include "openvmp_control_interactive/mode.hpp"
#include "rclcpp/rclcpp.hpp"

namespace openvmp_control_interactive {

class WalkMode : public ModeImpl {
 public:
  WalkMode() : ModeImpl(WALK, "Walk") {}
  virtual void enter(Mode from) override;
  virtual void leave(Mode to) override;
};

}  // namespace openvmp_control_interactive

#endif  // OPENVMP_CONTROL_INTERACTIVE_MODE_WALK_H
