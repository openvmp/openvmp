/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-12-10
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_CONTROL_INTERACTIVE_CONTROL_H
#define OPENVMP_CONTROL_INTERACTIVE_CONTROL_H

#include <memory>
#include <string>

#include "openvmp_control_interactive/mode.hpp"

namespace openvmp_control_interactive {

enum Control { CONTROL_NONE = 0, TRAJ_VEL, POSITION };

class ControlImpl : public ModeImpl {
 public:
  ControlImpl(
      rclcpp::Node *node,
      std::shared_ptr<interactive_markers::InteractiveMarkerServer> server,
      Mode mode, const std::string &name)
      : ModeImpl(node, server, mode, name) {}

  virtual Control get_control() const = 0;
  virtual void init() = 0;
  virtual void fini() = 0;

  virtual void enter(std::shared_ptr<ControlImpl> from) = 0;
  virtual void leave(std::shared_ptr<ControlImpl> to) = 0;
};

class Modes : public std::map<Mode, std::shared_ptr<ControlImpl>> {
 public:
  void add(std::shared_ptr<ControlImpl> impl) {
    insert({impl->get_mode(), impl});
  }
};

}  // namespace openvmp_control_interactive

#endif  // OPENVMP_CONTROL_INTERACTIVE_CONTROL_H
