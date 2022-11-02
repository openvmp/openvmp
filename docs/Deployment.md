# OpenVMP

## Deployment

### Prerequisites

- Raspberry PI 4
- SD Card with Ubuntu 22.04 or higher (64bit arm)
    - `/root/.ssh/authorized` contains your public key
    - `/etc/ssh/sshd_config` has `PermitRootLogin` uncommented
    - Your favorite flavor of ROS2 installed
- Local Ansible setup
    - `/etc/ansible/hosts` containing

        ```
        [openvmp-pi-unconfigured]
        root@<<<put your Raspberry PI Ethernet IP here>>>
        ```


### Initial setup

Use the following commands to start the deployment:

```
$ cd deployment/ansible
$ ansible-playbook ./setup.yml
```


### Post-setup

Once the setup is finished, 
create new entries in the Ansible inventory using the WiFi IP addresses
in the `openvmp-pi` section.
That's the inventory section used by all other OpenVMP Ansible playbooks.

```
        [openvmp-pi]
        root@<<<put your Raspberry PI WiFi IPs here>>>
```


### Configuration

Use the following commands to configure the board for the first time
as well as to update the configuration on each repository update
(or whenever needed for any other reason):

```
$ cd deployment/ansible
$ ansible-playbook ./configure.yml
```


![ROS/ROS2 index package for OpenVMP module: Deployment](https://www.google-analytics.com/collect?v=1&tid=UA-242596187-2&cid=555&aip=1&t=event&ec=github&ea=md&dp=%2Deployment.md&dt=OpenVMP%20Documentation)