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

![ROS/ROS2 index package for OpenVMP module: Deployment](https://www.google-analytics.com/collect?v=1&tid=UA-242596187-2&cid=555&aip=1&t=event&ec=github&ea=md&dp=%2Deployment.md&dt=OpenVMP%20Documentation)