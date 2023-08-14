
# OpenVMP

## Visual interactive control

The easiest way to control OpenVMP robots is by using
[interactive markers](http://wiki.ros.org/rviz/Tutorials/Interactive%20Markers%3A%20Getting%20Started) in RViz.

## Motion demo scripts

OpenVMP robots can also be controlled by using demo scripts that perform specific motion commands:

```sh
ros2 run openvmp_robot motion_demo_stand.py
```

```sh
ros2 run openvmp_robot motion_demo_walk.py
```

## Tasks and missions

OpenVMP units will be controlled by tasks and missions in the future.
However this functionality is still being developed and not yet released.
