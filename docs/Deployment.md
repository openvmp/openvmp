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
root@<<<put your Raspberry PI WiFi IPs here>>>
```

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

### Building

Use the following commands to update the source code repository on the target
and to build all packages:

```sh
cd deployment/ansible
ansible-playbook ./build.yml
```
