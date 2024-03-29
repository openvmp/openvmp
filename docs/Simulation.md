# OpenVMP

## Select the robot to simulate

The list of OpenVMP robot "kinds" available can be found using the following commands:

```bash
cd platform/src
ls -d openvmp_robot_*`
```

The recommended robot kind to start with is `don1`.

Disclaimer: it's the only robot kind published at the moment anyway.

## Select the world to simulate

The list of worlds available can be found using the following commands:

```bash
cd platform/src/openvmp_robot/worlds
ls *.world
```

There recommended world to start with is `mini`.
Other worlds have higher hardware demands for the simulation hots.

## Select the level of robot detail

With the available hardware resources (mostly CPU and RAM) in mind,
the following options should be considered:

| none | low | high |
| --- | --- | --- |
| `use_meshes:=none` | `use_meshes:=low` | `use_meshes:=high` |
| default, if [models](../models/) are NOT rendered | default, if [models](../models/) are rendered | not recommended yet |
| <img alt=none src="images/meshes/none.png" /> | <img alt=low src="images/meshes/low.png" /> | <img alt=high src="images/meshes/high.png" /> |

Use the parameter `use_meshes` in the lauch commands below.
By default, `use_meshes` is either `none` or `low`,
depending on whether the [models](../models/) are rendered or not.
Only `none` is available until the [models](../models/) are rendered.

## Launch the simulation

In order to simulate the chosen robots in the chosen world,
use the following command:

```bash
cd platform
ros2 launch openvmp_robot simulation_world.launch.py \
  world:=<world-name> \
  kind:=<robot-kind> \
  num:=<number-of-units-to-spawn> \
  use_meshes:=<level-of-detail>
```

or, to simulate a single robot `don1`
in the default world `mini`, run:

```bash
cd platform
ros2 launch openvmp_robot simulation_world.launch.py
```

Alternatively, use the following command to simulate the motion control stack
similar to the one used on the real robot
(instead of `gazebo_ros2_control`, a custom controller manager is created inside
Gazebo which uses `remote_hardware_interface` and knows how to simulate brakes):

```bash
cd platform
ros2 launch openvmp_robot simulation_world.launch.py simulate_remote_hardware_interface:=true
```

Please, note, the first simulation run takes a long time to download all
the necessary assets.
To preload the assets ahead of time,
use `cd platform && gazebo src/openvmp_robot/worlds/mini.world`.
Then exit and proceed with the commands shown above.

In case you are using a VM, make sure that graphics hardware acceleration is ON and working optimally (no `llvmpipe` and stuff).

<!-- 

## Spawn the robots manually

Far advanced users, there may be a desire to spawn robots manually.

First, launch the simulated world without spawning the robots automatically.
That can be done using the following command:

```
ros2 launch openvmp_robot simulation_world.launch.py world:=<world-name>
```

or, to launch the default world (`world:=horizontal`), simply run:

```
ros2 launch openvmp_robot simulation_world.launch.py
```

Once the simulation is running, OpenVMP units can be spawn manually
using the `Insert` tab to insert any model that starts with `OpenVMP: `.
Once the robot appears, the corresponding (specific to the model of the unit inserted) launch script has to be used to spawn all ROS2 nodes required for its operation. In case of `OpenVMP: don1`, the following command should be used:

```
ros2 launch openvmp_robot_don1 robot.py id:=<id-of-the-robot>
```

where `<id-of-the-robot>` could be seen in the Gazebo UI at the end of the name of the spawned entity. For example, the ID of `openvmp_robot_don1_XYZ1` is `XYZ1`.

-->

## What do I see?

Once the simulation is running,
there are several windows displayed on the screen.

One of the windows displayed is called Gazebo (or "gzclient").
This window shows the world that is being simulated.
This window always shows "the truth".

There is also one RViz window displayed for each robot that is spawned.
These windows show the perception of the world according
to each particular robot.
While, ideally, these windows show "the truth" as well, it's not always like that. Detecting the deviation between the reality and the robot's perception of the reality is one of the objectives of simulation.
And it's always going to be just a fraction of the world
that the robot is aware of.

Depending on the state of the robot, RViz windows may visualize
interactive controls to manually move the joints of the robot.

## What do I do next?

Once the simulation is running, each spawned OpenVMP unit can be controlled independently.

See [Control.md](./Control.md) for more details.

## Disclaimers

Please, note, the first simulation run of each new world is going to take some time to start as Gazebo needs to download all of the required models.

The lifecycle management of of all components involved in OpenVMP simulation
still leaves much to be desired. It may sometimes take more than one attempt
to launch the simulation successfully.

Apologies for the inconveniences and stay tuned for future updates.
