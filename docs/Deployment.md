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
        [pi]
        root@<<<RaspberryPI-IP>>>
        ```


### Launch

Use the following command to start the deployment:

```
ansible-playbook ansible/setup.yml
```
