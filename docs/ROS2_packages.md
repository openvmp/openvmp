# OpenVMP

## ROS2 packages

The below non-third-party packages are included in the OpenVMP mono-repo.

### Generic packages (not OpenVMP specific)

- [serial](https://github.com/openvmp/serial)

  Ultimate serial port driver for ROS2.
  Supports data flow introspection and mocking.

  There is a native library interface to build other packages/drivers on top of it. See [modbus\_rtu](https://github.com/openvmp/modbus_rtu) as an example.

- [modbus](https://github.com/openvmp/modbus)

  Universal Modbus driver for ROS2.
  Support for both client and server behaviors.

  This library is supposed to be used by [modbus\_rtu](https://github.com/openvmp/modbus_rtu) and [modbus\_tcp](https://github.com/openvmp/modbus_tcp).

  There is also a native interface to build drivers for specific Modbus devices.
  See [stepper\_driver\_rs485\_so](https://github.com/openvmp/stepper_driver_rs485_so) as an example. Remarkable features include the generation of ROS2 services for access to all Modbus registers described in a device specific YAML file.

- [modbus\_rtu](https://github.com/openvmp/modbus_rtu)

  Universal Modbus RTU driver for ROS2.
  It implements [the Modbus interface](https://github.com/openvmp/modbus).

  It allows sending arbitrary ad-hoc commands to Modbus RTU connected devices.  It is also a native library interface to build other packages/drivers on top of it. See [stepper\_driver\_rs485\_so](https://github.com/openvmp/stepper_driver_rs485_so) as an example.

- [stepper\_driver](https://github.com/openvmp/stepper_driver)

  Stepper driver module for ROS2. It implements a generic interface that is sufficient to drive stepper motors using ROS2 methods (ros2\_control etc) yet abstract enough to be implemented in device specific packages in separate repositories.

  This is a native library interface to build other packages/drivers on top of it. See [stepper\_driver\_rs485\_so](https://github.com/openvmp/stepper_driver_rs485_so) as an example.

- [stepper\_driver\_rs485\_so](https://github.com/openvmp/stepper_driver_rs485_so)

  Driver for STEPPERONLINE DM556RS and DM882RS stepper drivers.
  In addition to [the generic stepper driver interface](https://github.com/openvmp/stepper_driver), this module exposes ROS2 services and topics that are specific to this particular stepper driver, providing its entire feature set for other ROS2 nodes to use.

  This is a native library interface to build other packages/drivers on top of it. See [stepper\_driver\_rs485\_so](https://github.com/openvmp/stepper_driver_rs485_so) as an example.

- [stepper\_driver\_terminal](https://github.com/openvmp/stepper_driver_terminal)

  A terminal application to control any stepper driver that has a driver module implementing [the generic stepper driver interface](https://github.com/openvmp/stepper_driver).

### OpenVMP specific packages

TODO

![ROS/ROS2 index package for OpenVMP module: ROS2 packages](https://www.google-analytics.com/collect?v=1&tid=UA-242596187-2&cid=555&aip=1&t=event&ec=github&ea=md&dp=%2ROS2_packages.md&dt=ROS2%20package%20index)
