# OpenVMP

## ROS2 packages

The below non-third-party packages are included in the OpenVMP mono-repo.

### Generic packages (not OpenVMP specific)

The below list consists of packages that were introduced during OpenVMP project
and does not include third-party libraries that are present in the non-repo
(e.g. yaml-cpp).

- [serial](https://github.com/openvmp/serial)

  Ultimate serial port driver for ROS2.
  Supports data flow introspection and mocking.

  There is a native library interface to build other packages/drivers on top of
  it. See [modbus\_rtu](https://github.com/openvmp/modbus_rtu) as an example.

- [modbus](https://github.com/openvmp/modbus)

  Universal Modbus driver for ROS2.
  Support for both client and server behaviors.

  This library is supposed to be used by
  [modbus\_rtu](https://github.com/openvmp/modbus_rtu) and
  [modbus\_tcp](https://github.com/openvmp/modbus_tcp).

  There is also a native interface to build drivers for specific Modbus devices.
  See
  [stepper\_driver\_rs485\_so](https://github.com/openvmp/stepper_driver_rs485_so)
  as an example.
  Remarkable features include the generation of ROS2 services for access to all
  Modbus registers described in a device specific YAML file.

- [modbus\_rtu](https://github.com/openvmp/modbus_rtu)

  Universal Modbus RTU driver for ROS2.
  It implements [the Modbus interface](https://github.com/openvmp/modbus).

  It allows sending arbitrary ad-hoc commands to Modbus RTU connected devices.
  It is also a native library interface to build other packages/drivers on top
  of it. See
  [stepper\_driver\_rs485\_so](https://github.com/openvmp/stepper_driver_rs485_so)
  as an example.

- [brake](https://github.com/openvmp/brake)

  Brake module for ROS2. It implements a generic interface for joint brakes.
  It supports brakes that are engaged by default (such as integrated electromagnetic brakes for stepper motors) as well as brakes that are engaged on demand only.

- [stepper\_driver](https://github.com/openvmp/stepper_driver)

  Stepper driver module for ROS2. It implements a generic interface that is
  sufficient to drive stepper motors using ROS2 methods (ros2\_control etc) yet
  abstract enough to be implemented in device specific packages in separate
  repositories.

  This is a native library interface to build other packages/drivers on top of
  it. See
  [stepper\_driver\_rs485\_so](https://github.com/openvmp/stepper_driver_rs485_so)
  as an example.

- [stepper\_driver\_rs485\_so](https://github.com/openvmp/stepper_driver_rs485_so)

  Driver for STEPPERONLINE DM556RS and DM882RS stepper drivers.
  In addition to
  [the generic stepper driver interface](https://github.com/openvmp/stepper_driver),
  this module exposes ROS2 services and topics that are specific to this
  particular stepper driver, providing its entire feature set for other ROS2
  nodes to use.

  This is a native library interface to build other packages/drivers on top of
  it. See
  [stepper\_driver\_rs485\_so](https://github.com/openvmp/stepper_driver_rs485_so)
  as an example.

- [stepper\_driver\_terminal](https://github.com/openvmp/stepper_driver_terminal)

  A terminal application to control any stepper driver that has a driver module
  implementing
  [the generic stepper driver interface](https://github.com/openvmp/stepper_driver).

### OpenVMP specific packages

- [openvmp\_robot\_don1](https://github.com/openvmp/openvmp_robot_don1)

  The very first robot created in OpenVMP. It is big and capable. It will either
  get replaced by other robots eventually or it will morph into a swarm leader
  robot.

