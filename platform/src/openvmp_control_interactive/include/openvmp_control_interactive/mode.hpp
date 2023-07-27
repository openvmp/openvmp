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

#include "interactive_markers/interactive_marker_server.hpp"

namespace openvmp_control_interactive {

enum Mode {
  NONE = 0,
  FULL,
  WALK,
  DRIVE,
  HUG,
  HANG,
  GRAB,
  LIFT,
  CAMERAS_FRONT_LEFT,
  CAMERAS_FRONT_RIGHT,
  CAMERAS_REAR_LEFT,
  CAMERAS_REAR_RIGHT
};

class ModeImpl {
 public:
  ModeImpl(rclcpp::Node *node,
           std::shared_ptr<interactive_markers::InteractiveMarkerServer> server,
           Mode mode, const std::string &name)
      : node_{node}, server_{server}, mode_{mode}, name_{name} {}
  virtual ~ModeImpl() = default;

  Mode get_mode() const { return mode_; }
  const std::string &get_name() const { return name_; }
  virtual bool is_whole_body() const { return false; }
  virtual bool is_cameras() const { return false; }

 protected:
  std::mutex state_lock_;
  rclcpp::Node *const node_;
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;

 private:
  const Mode mode_;
  const std::string name_;
};

}  // namespace openvmp_control_interactive

#endif  // OPENVMP_CONTROL_INTERACTIVE_MODE_H
