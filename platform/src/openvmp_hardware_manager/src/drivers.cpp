/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-16
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "openvmp_hardware_manager/drivers.hpp"

#include "amt21_driver/factory.hpp"
#include "em2rs_driver/factory.hpp"
#include "remote_actuator/fake_factory.hpp"
#include "remote_actuator_pwm/factory.hpp"
#include "remote_brake/fake_factory.hpp"
#include "remote_brake_switch/factory.hpp"
#include "remote_encoder/fake_factory.hpp"
#include "remote_microcontroller/factory.hpp"
#include "remote_modbus_rtu/factory.hpp"
#include "remote_serial_bus/factory.hpp"

namespace openvmp_hardware_manager {

std::map<std::string, Drivers> driver_classes = {
    {
        "bus",
        {
            {"microcontroller",
             {
                 .factory =
                     [](rclcpp::Node *node,
                        std::shared_ptr<
                            rclcpp::executors::MultiThreadedExecutor>
                            exec) {
                       return remote_microcontroller::Factory::New(node, exec);
                     },
                 .params = {{"microcontroller_prefix",
                             YAML::Node(std::string("$PATH"))}},
                 .init = {},
             }},
            {"modbus_rtu",
             {
                 .factory =
                     [](rclcpp::Node *node,
                        std::shared_ptr<
                            rclcpp::executors::MultiThreadedExecutor>) {
                       return remote_modbus_rtu::Factory::New(node);
                     },
                 .params = {{"modbus_prefix",
                             YAML::Node(std::string("$PATH"))}},
                 .init = {},
             }},
            {"serial_bus",
             {
                 .factory =
                     [](rclcpp::Node *node,
                        std::shared_ptr<
                            rclcpp::executors::MultiThreadedExecutor>) {
                       return remote_serial_bus::Factory::New(node);
                     },
                 .params = {{"serial_bus_prefix",
                             YAML::Node(std::string("$PATH"))}},
                 .init = {},
             }},
        },
    },
    {
        "brake",
        {
            {"fake",
             {
                 .factory =
                     [](rclcpp::Node *node,
                        std::shared_ptr<
                            rclcpp::executors::MultiThreadedExecutor>) {
                       return remote_brake::FakeFactory::New(node);
                     },
                 .params = {{"brake_prefix", YAML::Node(std::string("$PATH"))}},
                 .init = {},
             }},
            {"switch",
             {
                 .factory =
                     [](rclcpp::Node *node,
                        std::shared_ptr<
                            rclcpp::executors::MultiThreadedExecutor>) {
                       return remote_brake_switch::Factory::New(node);
                     },
                 .params = {{"brake_prefix", YAML::Node(std::string("$PATH"))}},
                 .init = {},
             }},
        },
    },
    {
        "encoder",
        {
            {"fake",
             {
                 .factory =
                     [](rclcpp::Node *node,
                        std::shared_ptr<
                            rclcpp::executors::MultiThreadedExecutor>) {
                       return remote_encoder::FakeFactory::New(node);
                     },
                 .params = {{"encoder_prefix",
                             YAML::Node(std::string("$PATH"))}},
                 .init = {},
             }},
            {"amt21",
             {
                 .factory =
                     [](rclcpp::Node *node,
                        std::shared_ptr<
                            rclcpp::executors::MultiThreadedExecutor>) {
                       return amt21_driver::Factory::New(node);
                     },
                 .params = {{"encoder_prefix",
                             YAML::Node(std::string("$PATH"))}},
                 .init = {},
             }},
        },
    },
    {
        "actuator",
        {
            {"fake",
             {
                 .factory =
                     [](rclcpp::Node *node,
                        std::shared_ptr<
                            rclcpp::executors::MultiThreadedExecutor>) {
                       return remote_actuator::FakeFactory::New(node);
                     },
                 .params = {{"actuator_prefix",
                             YAML::Node(std::string("$PATH"))}},
                 .init = {},
             }},
            {"em2rs",
             {
                 .factory =
                     [](rclcpp::Node *node,
                        std::shared_ptr<
                            rclcpp::executors::MultiThreadedExecutor>) {
                       return em2rs_driver::Factory::New(node);
                     },
                 .params = {{"actuator_prefix",
                             YAML::Node(std::string("$PATH"))},
                            {"stepper_prefix",
                             YAML::Node(std::string("$PATH/stepper"))}},
                 .init = {},
             }},
            {"pwm",
             {
                 .factory =
                     [](rclcpp::Node *node,
                        std::shared_ptr<
                            rclcpp::executors::MultiThreadedExecutor>) {
                       return remote_actuator_pwm::Factory::New(node);
                     },
                 .params = {{"actuator_prefix",
                             YAML::Node(std::string("$PATH"))}},
                 .init = {},
             }},
        },
    },
};

}  // namespace openvmp_hardware_manager
