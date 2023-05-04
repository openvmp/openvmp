# OpenVMP

[![License](docs/license.svg)](./LICENSE)

Version: Alpha 1

![OpenVMP rendered robot model](./docs/images/front.png)

![walking robot](./docs/images/walk.png)
![driving robot](./docs/images/drive.png)
![remotely controlled robot](./docs/images/control.png)

![pole climbing robot](./docs/images/hug.png)
![cable climbing robot](./docs/images/hang.png)
![robot modes of operation](./docs/images/modes.png)

![daisy chained robots](./docs/images/chain.png)
![grab and attach to objects](./docs/images/grab.png)
![robot swarm](./docs/images/swarm.png)

**Open Versatile Mobility Platform** (**OpenVMP**)
implements multi-modal mobility mechanisms
that perform arbitrary tasks individually or collectively.

This is the monorepo of the OpenVMP project.
Some of the internal components are git submodules
that are also designed to be equally usable as standalone ROS2 packages
outside of OpenVMP
(unless the name of the package starts with `openvmp_`).

## Key features
### Multi-Modal Mobility
OpenVMP provides support for robots with different modes of transportation, including:

Driving on or off-road using wheels
Walking or crawling on four legs
Climbing trees, poles, pipes, ropes, cables, and other near-cylindrical objects
Climbing and driving inside pipes and tunnels
Climbing warehouse shelving units
### Individual Performance
OpenVMP robots can use the same limbs to perform various mechanical tasks, and their true power lies in the use of extension modules (see 'Modularity' below). Single OpenVMP units can operate independently for extended periods, with software support for minimizing power usage while idle. A key use case is individual OpenVMP units temporarily leaving the collective to perform ad-hoc tasks such as reconnaissance, communication, supply, and delivery.

### Collective Performance
Multiple OpenVMP units can share resources for computation and communication, as well as mechanically join together for improved mobility. The collective performance of versatile mobility units is evident in scenarios such as:

Climbing artificial structures (e.g., buildings, industrial complexes, complex pipes and tubes, poles, fences, barricades)
Overcoming natural barriers (e.g., trees, ravines, small cliffs)
### Modularity
Each OpenVMP unit may have one or more payload modules, typically including:

Front and rear (interchangeable) modules: Enables functional tasks with multipurpose or specialized tools (e.g., mechanical tools, inspection tools, liquid dispensers, defense mechanisms). These modules are hot-swappable.
One top module: Enhances swarm capabilities (e.g., advanced computing units, large robotic arms, extra power supplies). This module is hardwired and not hot-swappable.



## What's included

This repository contains all the software required to simulate OpenVMP units with a number of simulation worlds created specifically to demonstrate certain (but not yet all!) capabilities.

This repository also includes some of the software required to operate real OpenVMP units. The rest of the software (as well as the hardware blueprints for a number of different types of OpenVMP units that are currently being built) will also be published here soon. Stay tuned!

All materials in all OpenVMP repositories are published using the Apache 2.0 open-source license. To prevent patent trolls from making claims for minor improvements on top of the information published by OpenVMP, [the claims](docs/Claims.md) are made about the features and inventions that have already been considered and planned by OpenVMP contributors.

## More information

See the following documents for more info:

- [How to build OpenVMP software](docs/Development.md)
- [How to run a simulation](docs/Simulation.md)
- [How to build a real OpenVMP robot](docs/Hardware.md)
- [How to deploy to a real robot](docs/Deployment.md)
- [Included ROS2 packages](docs/ROS2_packages.md)
- [Roadmap](docs/Roadmap.md)
- [License](docs/License.md)

