/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-16
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "openvmp_hardware_manager/drivers.hpp"

#include "brake/fake_factory.hpp"
#include "brake_switch/factory.hpp"
#include "encoder_amt21/factory.hpp"
#include "modbus_rtu/factory.hpp"
#include "remote_actuator/fake_factory.hpp"
#include "remote_encoder/fake_factory.hpp"
#include "serial_bus/factory.hpp"
#include "stepper_driver_em2rs/factory.hpp"

namespace openvmp_hardware_manager {

std::map<std::string, Drivers> driver_classes = {
    {
        "bus",
        {
            {"modbus_rtu",
             {
                 .factory = [] (rclcpp::Node *node) { return modbus_rtu::Factory::New(node); },
                 .params = {{"modbus_prefix",
                             YAML::Node(std::string("$PATH"))}},
                 .init = {},
             }},
            {"serial_bus",
             {
                 .factory = [] (rclcpp::Node *node) { return serial_bus::Factory::New(node); },
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
                 .factory = [] (rclcpp::Node *node) { return brake::FakeFactory::New(node); },
                 .params = {{"brake_prefix", YAML::Node(std::string("$PATH"))}},
                 .init = {},
             }},
            {"switch",
             {
                 .factory = [] (rclcpp::Node *node) { return brake_switch::Factory::New(node); },
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
                     [] (rclcpp::Node *node) { return remote_encoder::FakeFactory::New(node); },
                 .params = {{"encoder_prefix",
                             YAML::Node(std::string("$PATH"))},
                            {"encoder_readings_per_second",
                             YAML::Node(std::string("10.0"))}},
                 .init = {},
             }},
            {"amt21",
             {
                 .factory = [] (rclcpp::Node *node) { return encoder_amt21::Factory::New(node); },
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
                     [] (rclcpp::Node *node) { return remote_actuator::FakeFactory::New(node); },
                 .params = {{"actuator_prefix",
                             YAML::Node(std::string("$PATH"))}},
                 .init = {},
             }},
            {"em2rs",
             {
                 .factory =
                     [] (rclcpp::Node *node) { return stepper_driver_em2rs::Factory::New(node); },
                 .params = {{"actuator_prefix",
                             YAML::Node(std::string("$PATH"))},
                            {"stepper_prefix",
                             YAML::Node(std::string("$PATH/stepper"))}},
                 .init = {},
             }},
        },
    },
};

}  // namespace openvmp_hardware_manager
