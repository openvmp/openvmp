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
               const std::string &id, const std::string &path_,
               std::shared_ptr<openvmp_hardware_configuration::Driver> config,
               bool use_fake_hardware)
    : exec_{exec} {
  RCLCPP_INFO(parent->get_logger(), "Initializing the driver: %s: %s: %s",
              driver_class.c_str(), node_name.c_str(), id.c_str());

  if (use_fake_hardware) {
    config.reset();
  }

  auto ns = parent->get_namespace();
  std::string path = path_;
  if (path == "") {
    path = "/" + driver_class + "/" + id;
  }
  path = std::regex_replace(path, std::regex(R"(\$DRIVER_NAME)"), id);
  // RCLCPP_INFO(parent->get_logger(), "Driver's path %s", path.c_str());
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
      RCLCPP_INFO(parent->get_logger(),
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
  // RCLCPP_INFO(parent->get_logger(), "launching the node %s",
  // node_name.c_str());
  node_options.use_intra_process_comms(true);
  node_ = std::make_shared<rclcpp::Node>(node_name, ns, node_options);
  exec_->add_node(node_);
  // RCLCPP_INFO(parent->get_logger(), "done launching the node %s",
  //             node_name.c_str());

  instance_ = driver_info.factory(node_.get());
  // RCLCPP_INFO(parent->get_logger(), "created the interface");

  // RCLCPP_INFO(parent->get_logger(), "initialization steps: %lu",
  // init.size());
  for (auto &step : init) {
    // RCLCPP_INFO(parent->get_logger(), "initialization step");
    if (step.type == "ros2_modbus/srv/ConfiguredHoldingRegisterWrite") {
      rclcpp::Client<ros2_modbus::srv::ConfiguredHoldingRegisterWrite>::
          SharedPtr clnt_modbus_chrw;

      auto service_path = ns + path + step.service;
      // RCLCPP_INFO(parent->get_logger(), "connecting to the service %s",
      //             service_path.c_str());

      clnt_modbus_chrw =
          node_
              ->create_client<ros2_modbus::srv::ConfiguredHoldingRegisterWrite>(
                  service_path);
      clnt_modbus_chrw->wait_for_service();

      auto request = std::make_shared<
          ros2_modbus::srv::ConfiguredHoldingRegisterWrite::Request>();
      request->value = step.fields["value"].as<int>();

      auto f = clnt_modbus_chrw->async_send_request(request);
      f.wait();

      auto _response = f.get();
      // RCLCPP_INFO(parent->get_logger(), "response to init request received:
      // %d",
      //             response->exception_code);
    } else {
      RCLCPP_ERROR(parent->get_logger(),
                   "Unsupported initialization step type: %s",
                   step.type.c_str());
    }
  }
  // RCLCPP_INFO(parent->get_logger(), "Done initializing the driver: %s: %s:
  // %s",
  //             driver_class.c_str(), node_name.c_str(), id.c_str());
}

}  // namespace openvmp_hardware_manager
