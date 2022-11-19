# OpenVMP

## Select the robot to simulate

The list of OpenVMP unit kinds available can be found using the following commands:

```
cd platform/src
ls openvmp_robot_*`
```

There recommended robot kind to start with is `don1`. 
Disclaimer: it's the only robot published at the moment.

## Select the world to simulate

The list of worlds available can be found using the following commands: 

```
cd platform/src/openvmp_robot/worlds
ls *.world
```

There recommended world to start with is `horizontal`.

## Launch the simulation

In order to simulate the chosen robots in the chosen world,
use the following command:

```
ros2 launch openvmp_robot simulation_world.launch.py \
  world:=<world-name> \
  kind:=<robot-kind> \
  num:=<number-of-units-to-spawn>
```

or, to simulate a single robot `don1`
in the default world `horizontal`, run:

```
ros2 launch openvmp_robot simulation_world.launch.py
```
<!-- 

## Spawn the robots manually

Far advanced users, there may be a desire to spawn robots manually.

First, launch the simulated world without spawning the robots automatically.
That can be done using the following command:

```
ros2 launch openvmp_robot simulation_world.py world:=<world-name>
```

or, to launch the default world (`world:=horizontal`), simply run:

```
ros2 launch openvmp_robot simulation_world.py
```

Once the simulation is running, OpenVMP units can be spawn manually
using the `Insert` tab to insert any model that starts with `OpenVMP: `.
Once the robot appears, the corresponding (specific to the model of the unit inserted) launch script has to be used to spawn all ROS2 nodes required for its operation. In case of `OpenVMP: don1`, the following command should be used:

```
ros2 launch openvmp_robot_don1 robot.py id:=<id-of-the-robot>
```

where `<id-of-the-robot>` could be seen in the Gazebo UI at the end of the name of the spawned entity. For example, the ID of `openvmp_robot_don1_XYZ1` is `XYZ1`.

-->

## What do I see

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

## What do I do next

Once the simulation is running, each spawned OpenVMP unit can be controlled independently.

See [Control.md](./Control.md) for more details.

## Disclaimers

Please, note, the first simulation run of each new world is going to take some time to start as Gazebo needs to download all of the required models.

The lifecycle management of of all components involved in OpenVMP simulation
still leaves much to be desired. It may sometimes take more than one attempt
to launch the simulation successfully.

Apologies for the inconveniences and stay tuned for future updates.