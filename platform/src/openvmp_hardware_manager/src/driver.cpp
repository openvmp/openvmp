/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-17
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "openvmp_hardware_manager/driver.hpp"

#include <regex>

#include "openvmp_hardware_manager/drivers.hpp"

namespace openvmp_hardware_manager {

Driver::Driver(rclcpp::Node *parent,
               std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> exec,
               const std::string &driver_class, const std::string &node_name,
               const std::string &id,
               std::shared_ptr<openvmp_hardware_configuration::Driver> config,
               bool use_fake_hardware)
    : exec_{exec} {
  RCLCPP_DEBUG(parent->get_logger(), "Initializing the driver: %s: %s: %s",
               driver_class.c_str(), node_name.c_str(), id.c_str());

  if (use_fake_hardware) {
    config.reset();
  }

  auto ns = parent->get_namespace();
  auto path = "/" + driver_class + "/" + id;
  path = std::regex_replace(path, std::regex(R"(\$DRIVER_NAME)"), id);
  auto node_options = rclcpp::NodeOptions{};

  std::string type;
  if (config) {
    type = config->get_type();
  } else {
    type = "fake";
  }

  auto &dc = driver_classes.at(driver_class);
  auto driver_info_it = dc.find(type);
  if (driver_info_it == dc.end()) {
    if (type == "fake") {
      RCLCPP_DEBUG(parent->get_logger(),
                   "No need to instantiate a fake driver of class %s",
                   driver_class.c_str());
    } else {
      RCLCPP_ERROR(parent->get_logger(), "Unsupported driver %s of class %s",
                   type.c_str(), driver_class.c_str());
    }
    return;
  }
  auto &driver_info = driver_info_it->second;

  // First, add the standard parameters for this driver
  auto params = driver_info.params;
  auto init = driver_info.init;

  params.push_back({"use_remote", YAML::Node(std::string("false"))});

  // Then, add the parameters configured for this instance of the driver
  if (config) {
    auto config_params = config->get_params();
    auto config_init = config->get_init();
    params.insert(params.end(), config_params.begin(), config_params.end());
    init.insert(init.end(), config_init.begin(), config_init.end());
  }

  // Resolve the parameter values and add them to the node options
  for (auto &param : params) {
    try {
      auto value = param.second.as<int>();
      node_options.parameter_overrides().push_back({param.first, value});
    } catch (const std::exception &_e) {
      try {
        auto value = param.second.as<double>();
        node_options.parameter_overrides().push_back({param.first, value});
      } catch (const std::exception &_e) {
        try {
          auto value = param.second.as<bool>();
          node_options.parameter_overrides().push_back({param.first, value});
        } catch (const std::exception &_e) {
          auto value = param.second.as<std::string>();
          value = std::regex_replace(value, std::regex(R"(\$NAMESPACE)"), ns);
          value = std::regex_replace(value, std::regex(R"(\$DRIVER_NAME)"), id);
          value = std::regex_replace(value, std::regex(R"(\$PATH)"), path);

          node_options.parameter_overrides().push_back({param.first, value});
        }
      }
    }
  }

  // Create the node and make it spin
  node_ = std::make_shared<rclcpp::Node>(node_name, ns, node_options);
  exec_->add_node(node_);

  instance_ = driver_info.factory(node_.get());

  // FIXME(clairbee): perform initialization
  for (auto &step : init) {
    if (step.type == "modbus/srv/ConfiguredHoldingRegisterWrite") {
      if (!clnt_modbus_chrw_) {
        clnt_modbus_chrw_ =
            node_->create_client<modbus::srv::ConfiguredHoldingRegisterWrite>(
                step.service);
        clnt_modbus_chrw_->wait_for_service();
      }

      modbus::srv::ConfiguredHoldingRegisterWrite::Request::SharedPtr request;
      request->value = step.fields["value"].as<int>();

      auto f = clnt_modbus_chrw_->async_send_request(request);
      f.wait();
      RCLCPP_DEBUG(parent->get_logger(),
                   "configured_holding_register_write(): response received");

      modbus::srv::ConfiguredHoldingRegisterWrite::Response::SharedPtr
          _response = f.get();
    } else {
      RCLCPP_ERROR(parent->get_logger(),
                   "Unsupported initialization step type: %s",
                   step.type.c_str());
    }
  }
}

}  // namespace openvmp_hardware_manager