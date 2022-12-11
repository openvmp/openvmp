/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-12-10
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "openvmp_control_interactive/control.hpp"

namespace openvmp_control_interactive {

void ControlImpl::enter(std::shared_ptr<ControlImpl> from) {
  if (from->get_control() != get_control()) {
    init();
  }
}

void ControlImpl::leave(std::shared_ptr<ControlImpl> to) {
  if (to->get_control() != get_control()) {
    fini();
  }
}

}  // namespace openvmp_control_interactive
