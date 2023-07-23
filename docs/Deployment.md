# OpenVMP

## Deployment

To deploy OpenVMP software to robot hardware, you'll need a Linux or MacOS workstation. This will allow you to install and configure the necessary software components, as well as keep your robots up-to-date with the latest features and bug fixes.

### Workstation Prerequisites

The requirements must to be met:

- [Ansible](https://docs.ansible.com/ansible/latest/installation_guide/intro_installation.html) must be installed
- This repository must be cloned

### Target Prerequisites

#### Target Roles

OpenVMP robots contain one or more single board computers (SBC).

If there is more than one SBC present,
then one of them is dedicated to perform the role called "spinal". The other ones perform the role called "cerebral".

If there is only one SBC present, then it performs both "spinal" and "cerebral" roles at the same time.

#### Supported Target Hardware

OpenVMP deployment scripts support the following target hardware options.

| SBC                    | OS                       | Spinal | Cerebral | Alias  |
| ---------------------- | ------------------------ | ------ | -------- | ------ |
| Rock 5 Model B         | Armbian 23.0.2           | [x]    |          | rock5b |
| Raspberry Pi 4 Model B | Ubuntu 22.04 (64bit arm) | [x]    |          | pi4b   |
| Intel NUC              | Ubuntu 22.04             |        | [x]      | nuc    |

The hardware design of each particular robot dictates what SBCs can be
used for which roles.

| Robot Kind | Configuration | Spinal                                      | Cerebral                                                                               |
| ---------- | ------------- | ------------------------------------------- | -------------------------------------------------------------------------------------- |
| Don1       | Recommended   | Rock 5B 8GB+, eMMC 32GB+, Intel AX210/AX211 | 2x Intel NUC 12th Gen i5 32GB+, M.2 22x80 NVME SSD 512GB+, Coral AI M.2 22x30 Edge TPU |
| Don1       | Minimum       | Raspberry Pi 4 Model B 4GB, SD card 4GB     | Intel NUC 12th Gen i3 8GB, M.2 22x80 NVME SSD 256GB                                    |

### Configuration

#### Target Inventory

Each target board has to be registered in
`deployment/ansible/inventory50-sbcs.ini`.

```ini
# SBCs pending initial setup
[sbc-unconfigured-pi4b] # Raspberry Pi 4 Model B
<Ethernet-IP-of-pi4b>

[sbc-unconfigured-rock5b] # Rock 5 Model B
<Ethernet-IP-of-rock5b>

[sbc-unconfigured-nuc] # Intel NUC
<Ethernet-IP-of-NUC-1>
<Ethernet-IP-of-NUC-2>


# SBCs after the initial setup is complete
[sbc-spinal-pi4b] # Raspberry Pi 4 Model B
<WiFi-IP-of-pi4b>

[sbc-spinal-rock5b] # Rock 5 Model B
<WiFi-IP-of-rock5b>

[sbc-cerebral-nuc] # Intel NUC
# This example assumes that the NUCs are connected to the rock5b (see above).
<WiFi-IP-of-rock5b>:8021 spinal_sbc=<WiFi-IP-of-rock5b> index=1
<WiFi-IP-of-rock5b>:8022 spinal_sbc=<WiFi-IP-of-rock5b> index=2
```

Before the initial setup,
the target entries are placed in the section
`sbc-unconfigured-<SBC alias>`
using dynamically assigned Ethernet IP address (use `ip addr show`).

After the initial setup is completed,
the target entries are placed in the section `sbc-<role>-<SBC alias>`
using the WiFi IP address of the robot.
Please, make sure that the robot is assigned a static IP address
in the configuration of your WiFi access point (permanent IP lease)
for ease of remote management.

Please, note, if there are multiple SBCs per robot
then the robot gets a single IP address assigned to the "spinal" SBC.
Internally, "cerebral" SPCs are connected to the "spinal" one.
The "spinal" SBC is forwarding management traffic (SSH) to
the "cerebral" SBCs by forwarding ports 8021, 8022 etc...

*Please, note, the inventory examples in this document are
for illustration purposes only.
The actual inventory file will have a shorter list of target SBCs.
The exact list of entries depends on the specific requirements for
the robot kind (see above).*

#### Initial Setup

Each SBC has to be setup separately before being installed into the robot.

SBCs usually come with an OS pre-installed.
The chances are this is not an OS images that is supporte by OpenVMP.
Please, refer to a table above for the supported OS for each
particular SBC.

After the proper OS is installed, local user is configured and a dynamic
IP address is assigned on a wired Ethernet network, update the inventory file
(see above) and run the following commands:

```sh
cd deployment/ansible
ansible-playbook -i inventory ./setup.yaml
```

#### Installation

After the initial setup is completed, the target SBCs are
mounted to the robot. Refer to the assembly instructions for the details.

After the assembly, the cerebral SBCs (if any) are wired to the
corresponding spinal SBC. They are no longer exposed on the external
network.
Corresponding changes need to be made to the inventory file by moving
the entries to the section dedicated to SBCs that are already setup.

#### Finilizing and updating the target configuration

Use the following commands if you need to:

- finish configuring the target board for the first time after the initial setup
- link an additional development machine to an already configured target board
- update the target board software with latest ROS updates
- update the target board configuration on each major OpenVMP update
- fix build issues that may be caused by a misconfiguration or software version
  mismatch

```sh
cd deployment/ansible
ansible-playbook -i inventory ./configure.yaml
```

### Building software on targets

OpenVMP does not support cross-compilation at the moment.
Use the following commands to update the source code repository on
the target and to build it:

```sh
cd deployment/ansible
ansible-playbook -i inventory ./build.yaml
```

### Launching robot software

Use the following commands to run OpenVMP services on the target SBCs and
the teleoperation UI on the workstations:

```sh
cd deployment/ansible
ansible-playbook -i inventory ./run.yaml
```

### Connectivity between robots and development machines

Once some drivers and services are running on the target boards,
it's possible to interact with them from the development machine
if it's connected to the same WiFi network.
Use the following command on the development machine to confirm
that the robot is visible to ROS2 on the development machine:

```sh
ros2 node list
ros2 topic list
ros2 service list
```


The output of the above commands is one of the ways
to learn the IDs assigned to the target robots.

If there is no output produced at all then something is getting
in the way of ROS nodes on the workstation trying to reach to ROS nodes
on the target SBCs. The potential reasons for this include:
  - The workstation's firewall is blocking ROS2 connections.<br/>
    Consider disabling the firewall.
  - The WiFi access point has "client isolation" turned on.

### Testing robot software on the development machine

While the device drivers (and, potentially, other OpenVMP subsystems)
are running on the robot itself,
it's possible to run any robot subsystem on the development machine for
testing and troubleshooting purposes.

```sh
ros2 launch openvmp_robot robot.launch.py kind:=don1 id:=<<<robot_ID_goes_here>>> subsystem:=<<<robot_subsystem_name_goes_here>>>
```

### Controlling the robot from the command line

Test the motion control pipeline by running the following command
on the development machine:

```sh
ros2 service call /openvmp/robot_<<<robot_ID_goes_here>>>/joint_front_turn_table_joint_actuator/modbus/get_ppr modbus/srv/ConfiguredHoldingRegisterRead '{}'
```

The output should contain the current setting of 'Pulses Per Revolution'
of the stepper motor driver of the front turn table joint
(the one which turns the front limbs left and right relative to
the main robot body).

Use the following command to move some joints:

```sh
ros2 service call /openvmp/robot_<<<robot_ID_goes_here>>>/joint_front_turn_table_joint_actuator/velocity/set stepper_driver/srv/VelocitySet '{"velocity":2.0}'
```