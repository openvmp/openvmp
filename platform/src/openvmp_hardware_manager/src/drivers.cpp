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
                 .factory = (driver_factory_ptr)&modbus_rtu::Factory::New,
                 .params = {{"modbus_prefix",
                             YAML::Node(std::string("$PATH"))}},
             }},
            {"serial_bus",
             {
                 .factory = (driver_factory_ptr)&serial_bus::Factory::New,
                 .params = {{"modbus_prefix",
                             YAML::Node(std::string("$PATH"))}},
             }},
        },
    },
    {
        "brake",
        {
            {"fake",
             {
                 .factory = (driver_factory_ptr)&brake::FakeFactory::New,
                 .params = {{"brake_prefix", YAML::Node(std::string("$PATH"))}},
             }},
            {"switch",
             {
                 .factory = (driver_factory_ptr)&brake_switch::Factory::New,
                 .params = {{"brake_prefix", YAML::Node(std::string("$PATH"))}},
             }},
        },
    },
    {
        "encoder",
        {
            {"fake",
             {
                 .factory =
                     (driver_factory_ptr)&remote_encoder::FakeFactory::New,
                 .params = {{"encoder_prefix",
                             YAML::Node(std::string("$PATH"))},
                            {"encoder_readings_per_second",
                             YAML::Node(std::string("10.0"))}},
             }},
            {"switch",
             {
                 .factory = (driver_factory_ptr)&encoder_amt21::Factory::New,
                 .params = {{"encoder_prefix",
                             YAML::Node(std::string("$PATH"))}},
             }},
        },
    },
    {
        "actuator",
        {
            {"fake",
             {
                 .factory =
                     (driver_factory_ptr)&remote_actuator::FakeFactory::New,
                 .params = {{"actuator_prefix",
                             YAML::Node(std::string("$PATH"))}},
             }},
            {"em2rs",
             {
                 .factory =
                     (driver_factory_ptr)&stepper_driver_em2rs::Factory::New,
                 .params = {{"actuator_prefix",
                             YAML::Node(std::string("$PATH"))},
                            {"stepper_prefix",
                             YAML::Node(std::string("$PATH/stepper"))}},
             }},
        },
    },
};

}  // namespace openvmp_hardware_manager