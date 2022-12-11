/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-12-10
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "openvmp_control_interactive/mode_default.hpp"

namespace openvmp_control_interactive {

void DefaultMode::enter(Mode from) {
  (void)from;
  // transition from "from"
}

void DefaultMode::leave(Mode to) {
  (void)to;
  // transition to "to"
}

}  // namespace openvmp_control_interactive
