# OpenVMP

## Package for robot hardware specifications

Each OpenVMP robot has its hardware specifics detailed in a configuration file.
See [an example](../openvmp_robot_don1/config/hardware.yaml).

This package implements a parser and a container for such hardware configuration information.

This package is used in both
[simulation](../openvmp_hardware_simulation_gazebo/README.md) and
[runtime](../openvmp_hardware_manager/README.md) environments:

```mermaid
flowchart TB
    ohc[Canonical harfware specification:\nopenvmp_hardware_configuration]
    ohsg[Simulated hardware:\nopenvmp_hardware_simulation_gazebo]
    subgraph or[Real hardware: openvmp_robot]
        subgraph sd[Subsystem: drivers]
          ohm[openvmp_hardware_manager]
        end
    end
    subgraph mc[Subsystem: motion_control]
      omchw[hardware_interface::RobotHW\nopenvmp_motion_control_hardware]
    end

    ohc --> ohm
    ohc -.-> omchw
    omchw -.-> ohsg
    omchw -.-> ohm
    ohc --> ohsg
```
