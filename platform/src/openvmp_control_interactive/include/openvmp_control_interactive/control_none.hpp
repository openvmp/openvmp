/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-12-10
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_CONTROL_INTERACTIVE_CONTROL_NONE_H
#define OPENVMP_CONTROL_INTERACTIVE_CONTROL_NONE_H

#include <string>

#include "openvmp_control_interactive/control.hpp"

namespace openvmp_control_interactive {

class NoneControl : public ControlImpl {
 public:
  NoneControl(
      rclcpp::Node *node,
      std::shared_ptr<interactive_markers::InteractiveMarkerServer> server,
      Mode mode, const std::string &name)
      : ControlImpl(node, server, mode, name) {}

  virtual Control get_control() const override { return CONTROL_NONE; }
  virtual void init() override;
  virtual void fini() override;
};

}  // namespace openvmp_control_interactive

#endif  // OPENVMP_CONTROL_INTERACTIVE_CONTROL_NONE_H
