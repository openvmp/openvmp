/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-02-25
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_HARDWARE_SIMULATION_GAZEBO_DEVICE_NODE_H
#define OPENVMP_HARDWARE_SIMULATION_GAZEBO_DEVICE_NODE_H

#include <memory>
#include <string>

#include "openvmp_hardware_configuration/device.hpp"
#include "rclcpp/rclcpp.hpp"

namespace openvmp_hardware_simulation_gazebo {

template <typename INTF_TYPE, typename CONFIG_TYPE>
class DeviceNode : public rclcpp::Node {
 public:
  DeviceNode(const std::string &ns, const std::string &node_name,
             std::weak_ptr<Joint> joint, std ::shared_ptr<CONFIG_TYPE> config,
             const std::vector<rclcpp::Parameter> &params)
      : rclcpp::Node(node_name, ns,
                     rclcpp::NodeOptions()
                        .parameter_overrides(params)
                        .use_intra_process_comms(true)) {
    device = std::make_shared<INTF_TYPE>(this, joint, config);
  }
  virtual ~DeviceNode() {}

  std::shared_ptr<INTF_TYPE> device;
};

}  // namespace openvmp_hardware_simulation_gazebo

#endif  // OPENVMP_HARDWARE_SIMULATION_GAZEBO_DEVICE_NODE_H
