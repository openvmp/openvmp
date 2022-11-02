# OpenVMP

## Configuring the development environment

### Visual Studio Code

Run the following commands to have the support files generated for VSCode:

```
cd platform
COLCON_HOME=$(pwd) colcon build
```

## Building

### ROS2

In order to build and use all OpenVMP ROS2 packages, use the following commands:

```
cd platform
colcon build
./install/local_setup.bash # or .zsh
```

## Testing

### Testing prerequisites

The following tools need to be pre-installed on your OS:

- socat

![ROS/ROS2 index package for OpenVMP module: Development](https://www.google-analytics.com/collect?v=1&tid=UA-242596187-2&cid=555&aip=1&t=event&ec=github&ea=md&dp=%2FDevelopment.md&dt=OpenVMP%20Documentation)