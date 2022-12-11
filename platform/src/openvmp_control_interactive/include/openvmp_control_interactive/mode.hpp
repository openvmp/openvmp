/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-12-10
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_CONTROL_INTERACTIVE_MODE_H
#define OPENVMP_CONTROL_INTERACTIVE_MODE_H

#include <map>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

namespace openvmp_control_interactive {

enum Mode { DEFAULT, WALK, TURN };

class ModeImpl {
 public:
  ModeImpl(Mode mode, const std::string &name) : mode_{mode}, name_{name} {}
  virtual ~ModeImpl() = default;

  Mode get_mode() const { return mode_; }
  const std::string &get_name() const { return name_; }

  virtual void enter(Mode from) = 0;
  virtual void leave(Mode to) = 0;

 private:
  const Mode mode_;
  const std::string name_;
};

}  // namespace openvmp_control_interactive

#endif  // OPENVMP_CONTROL_INTERACTIVE_MODE_H
