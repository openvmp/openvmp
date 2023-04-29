/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-04-29
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "openvmp_hardware_simulation_gazebo/control.hpp"

#include <chrono>

using namespace std::chrono_literals;

namespace openvmp_hardware_simulation_gazebo {

Control::Control(rclcpp::Node *node,
                 std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> exec,
                 gazebo::physics::ModelPtr model)
    : node_{node}, model_{model} {
  // Get the Gazebo simulation period
  rclcpp::Duration gazebo_period(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(
              model_->GetWorld()->Physics()->GetMaxStepSize())));

  // Read urdf from ros parameter server then
  // setup actuators and mechanism control node.
  // This call will block if ROS is not properly initialized.
  std::string urdf_string;
  std::vector<hardware_interface::HardwareInfo> control_hardware_info;
  try {
    urdf_string = get_urdf_();
    control_hardware_info =
        hardware_interface::parse_control_resources_from_urdf(urdf_string);
  } catch (const std::runtime_error &ex) {
    RCLCPP_ERROR_STREAM(
        node_->get_logger(),
        "Error parsing URDF in gazebo_ros2_control plugin, plugin not active : "
            << ex.what());
    return;
  }

  std::unique_ptr<hardware_interface::ResourceManager> resource_manager =
      std::make_unique<hardware_interface::ResourceManager>();

  RCLCPP_DEBUG(node_->get_logger(), "Loading RemoteSystemInterface");
  try {
    robot_hw_sim_loader_.reset(
        new pluginlib::ClassLoader<
            remote_hardware_interface::RemoteSystemInterface>(
            "remote_hardware_interface",
            "remote_hardware_interface::RemoteSystemInterface"));
  } catch (pluginlib::LibraryLoadException &ex) {
    RCLCPP_ERROR(node_->get_logger(),
                 "Failed to create robot simulation interface loader: %s ",
                 ex.what());
    throw ex;
  }

  RCLCPP_DEBUG(node_->get_logger(), "Setting component state");
  for (unsigned int i = 0; i < control_hardware_info.size(); i++) {
    std::string robot_hw_sim_type_str =
        control_hardware_info[i].hardware_class_type;
    RCLCPP_INFO(node_->get_logger(), "Setting component state: %s",
                robot_hw_sim_type_str.c_str());
    if (control_hardware_info[i].hardware_parameters.find("namespace") ==
        control_hardware_info[i].hardware_parameters.end()) {
      control_hardware_info[i].hardware_parameters.at("namespace") =
          std::string(node_->get_namespace());
    }
    RCLCPP_INFO(
        node_->get_logger(), "Namespace: %s",
        control_hardware_info[i].hardware_parameters.at("namespace").c_str());

    auto remote_system =
        std::unique_ptr<remote_hardware_interface::RemoteSystemInterface>(
            robot_hw_sim_loader_->createUnmanagedInstance(
                robot_hw_sim_type_str));

    last_update_sim_time_ros_ = rclcpp::Time();
    // TODO(clairbee): initialize RemoteHardwareSystem?

    if (remote_system) {
      RCLCPP_INFO(node_->get_logger(), "Setting component state: %s: importing",
                  robot_hw_sim_type_str.c_str());
      resource_manager->import_component(std::move(remote_system),
                                         control_hardware_info[i]);

      // activate all components
      rclcpp_lifecycle::State state(
          lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
          hardware_interface::lifecycle_state_names::ACTIVE);
      RCLCPP_INFO(node_->get_logger(),
                  "Setting component state: %s: setting state",
                  robot_hw_sim_type_str.c_str());
      resource_manager->set_component_state(control_hardware_info[i].name,
                                            state);
    } else {
      RCLCPP_INFO(node_->get_logger(), "Failed to instantiate: %s",
                  robot_hw_sim_type_str.c_str());
    }
  }

  // Create the controller manager
  RCLCPP_INFO(node_->get_logger(), "Loading controller_manager");
  controller_manager_.reset(new controller_manager::ControllerManager(
      std::move(resource_manager), exec, "controller_manager",
      node_->get_namespace()));
  exec->add_node(controller_manager_);

  if (!controller_manager_->has_parameter("update_rate")) {
    RCLCPP_ERROR_STREAM(
        node_->get_logger(),
        "controller manager doesn't have an update_rate parameter");
    return;
  }

  auto cm_update_rate =
      controller_manager_->get_parameter("update_rate").as_int();
  control_period_ =
      rclcpp::Duration(std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(1.0 /
                                        static_cast<double>(cm_update_rate))));
  // Check the period against the simulation period
  if (control_period_ < gazebo_period) {
    RCLCPP_ERROR_STREAM(
        node_->get_logger(),
        "Desired controller update period ("
            << control_period_.seconds()
            << " s) is faster than the gazebo simulation period ("
            << gazebo_period.seconds() << " s).");
  } else if (control_period_ > gazebo_period) {
    RCLCPP_WARN_STREAM(
        node_->get_logger(),
        " Desired controller update period ("
            << control_period_.seconds()
            << " s) is slower than the gazebo simulation period ("
            << gazebo_period.seconds() << " s).");
  }
  // Force setting of use_sime_time parameter
  controller_manager_->set_parameter(
      rclcpp::Parameter("use_sim_time", rclcpp::ParameterValue(true)));

  // Listen to the update event. This event is broadcast every simulation
  // iteration.
  update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&Control::update_, this));
}

Control::~Control() {
  // Disconnect from gazebo events
  update_connection_.reset();

  controller_manager_.reset();
  robot_hw_sim_loader_.reset();
}

// Get the URDF XML from the parameter server
std::string Control::get_urdf_() const {
  const std::string param_name = "robot_description";
  const std::string robot_description_node_name =
      std::string(node_->get_namespace()) + "/robot_state_publisher";
  std::string urdf_string;

  using namespace std::chrono_literals;
  auto parameters_client = std::make_shared<rclcpp::AsyncParametersClient>(
      node_, robot_description_node_name);
  while (!parameters_client->wait_for_service(0.5s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node_->get_logger(),
                   "Interrupted while waiting for %s service. Exiting.",
                   robot_description_node_name.c_str());
      return 0;
    }
    RCLCPP_ERROR(node_->get_logger(),
                 "%s service not available, waiting again...",
                 robot_description_node_name.c_str());
  }

  RCLCPP_INFO(node_->get_logger(), "connected to service!! %s",
              robot_description_node_name.c_str());

  // search and wait for robot_description on param server
  while (urdf_string.empty()) {
    RCLCPP_DEBUG(node_->get_logger(), "param_name %s", param_name.c_str());

    try {
      auto f = parameters_client->get_parameters({param_name});
      f.wait();
      std::vector<rclcpp::Parameter> values = f.get();
      urdf_string = values.at(0).as_string();
    } catch (const std::exception &e) {
      RCLCPP_ERROR(node_->get_logger(), "%s", e.what());
    }

    if (!urdf_string.empty()) {
      break;
    } else {
      RCLCPP_ERROR(node_->get_logger(),
                   "gazebo_ros2_control plugin is waiting for model"
                   " URDF in parameter [%s] on the ROS param server.",
                   param_name.c_str());
    }
    usleep(100000);
  }
  RCLCPP_INFO(node_->get_logger(),
              "Recieved urdf from param server, parsing...");

  return urdf_string;
}

void Control::update_() {
  // Get the simulation time and period
  gazebo::common::Time gz_time_now = model_->GetWorld()->SimTime();
  rclcpp::Time sim_time_ros(gz_time_now.sec, gz_time_now.nsec, RCL_ROS_TIME);
  rclcpp::Duration sim_period = sim_time_ros - last_update_sim_time_ros_;

  if (sim_period >= control_period_) {
    controller_manager_->read(sim_time_ros, sim_period);
    controller_manager_->update(sim_time_ros, sim_period);
    last_update_sim_time_ros_ = sim_time_ros;
  }

  // Always set commands on joints, otherwise at low control frequencies the
  // joints tremble as they are updated at a fraction of gazebo sim time use
  // same time as for read and update call - this is how it is done is
  // ros2_control_node
  controller_manager_->write(sim_time_ros, sim_period);
}
void Control::reset_() {
  // Reset timing variables to not pass negative update periods to controllers
  // on world reset
  last_update_sim_time_ros_ = rclcpp::Time((int64_t)0, RCL_ROS_TIME);
}

}  // namespace openvmp_hardware_simulation_gazebo