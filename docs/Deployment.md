# OpenVMP

## Deployment

### Prerequisites

- Raspberry PI 4
- SD Card with Ubuntu 22.04 or higher (64bit arm)
  - `/root/.ssh/authorized` contains your public key
    (use `ssh-keygen` on the dev host to get one)
  - `/etc/ssh/sshd_config` has `PermitRootLogin` uncommented
  - Your favorite flavor of ROS2 installed
- Local Ansible setup
  - `/etc/ansible/hosts` containing

    ```ini
    [openvmp-pi-unconfigured]
    root@<<<put your Raspberry PI Ethernet IP here>>>
    ```

### Initial setup

Use the following commands to start the deployment:

```sh
cd deployment/ansible
ansible-playbook ./setup.yml
```

### Post-setup

Once the setup is finished,
create new entries in the Ansible inventory using the WiFi IP addresses
in the `openvmp-pi` section.
That's the inventory section used by all other OpenVMP Ansible playbooks.

```ini
[openvmp-pi]
root@<<<put your Raspberry PI WiFi IP here>>> robot_kind=don1
```

Please, note, `don1` is the only `robot_kind` supported by OpenVMP
at the moment.

### Configuration

Use the following commands if you need to:

- configure the target board for the first time
- link an additional development machine to an already configured target board
- update the target board software with latest ROS updates
- update the target board configuration on each major OpenVMP update
- fix build issues that may be caused by a misconfiguration or software version
  mismatch

```sh
cd deployment/ansible
ansible-playbook ./configure.yml
```

### Building software on the target

OpenVMP does not support cross-compilation at the moment.
Use the following commands to update the source code repository on the target
and to build all the packages:

```sh
cd deployment/ansible
ansible-playbook ./build.yml
```

### Launching the minimal set of services

Use the following commands to run OpenVMP device drivers software
on the target boards:

```sh
cd deployment/ansible
ansible-playbook ./run-minimal.yml
```

### Launching the full set of services

Use the following commands to run the complete set of OpenVMP software
on the target boards:

```sh
cd deployment/ansible
ansible-playbook ./run-full.yml
```

### Testing connectivity with development machines

Once some services are running on the target boards,
other robot subsystems can be launched and tested on the development machine.
Use the following command on the development machines to confirm
that the target machines is visible to ROS2 on the development machine:

```sh
ros2 node list
```

The output contains the IDs assigned to the target robots.

If there is no output, check the firewall settings on the development machine
to confirm that they allow ROS2 network connections.

### Controlling the robot remotely

Test the control pipeline by running the following command
on the development machine:

```sh
ros2 service call /openvmp/robot_<<<robot_ID_goes_here>>>/joint_front_turn_table_joint_actuator/modbus/get_ppr modbus/srv/ConfiguredHoldingRegisterRead '{}'
```

The output should contain the current setting of 'Pulses Per Revolution' of the
stepper motor driver of the front turn table joint
(the one which turns the front limbs left and right relative to the main robot
body).

### Testing robot software on the development machine

While the device drivers (and, potentially, other OpenVMP subsystems)
are running on the robot itself,
it's possible to run any robot subsystem on the development machine for
testing and troubleshooting purposes.

```sh
ros2 launch openvmp_robot robot.launch.py kind:=don1 id:=<<<robot_ID_goes_here>>> subsystem:=<<<robot_subsystem_name_goes_here>>>
```

The list of available subsystems may evolve over time.
The following subsystems should suffice to control the robot which has only device drivers running:

- `reflection`
- `motion_control`
- `teleop`

If the robot is running the full set of OpenVMP software then the only subsystem
required to control the robot remotely is `teleop`.
