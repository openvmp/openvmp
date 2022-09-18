# OpenVMP

## Configuring the development environment

### Visual Studio Code

Run the following commands to have the support files generated for VSCode:

```
cd platform
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

## Building

### ROS2

In order to build and use all OpenVMP ROS2 packages, use the following commands:

```
cd platform
colcon build
./install/local_setup.bash # or .zsh
```
