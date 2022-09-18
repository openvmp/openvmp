# OpenVMP

Open Versatile Mobility Platform (OpenVMP) implements multi-modal mobility mechanisms that perform arbitrary tasks collectively or individually.

The internal components of OpenVMP are designed to be usable in separate projects as well.

## Key features

### Multi-modal mobility

OpenVMP provides support for robots that can use different modes of transportation:

- driving on road or off road using wheels
- crawling on multiple points
- walking straight on two legs
- climbing trees, poles, pipes, ropes and cables
- climbing and driving inside pipes and tunnels 

### Collective performance

Multiple OpenVMP units can not only share their resources to achieve computation
and communication goals, but also join their bodies mechanically to improve
overall mobility performance. The enhanced mobility performance can be best seen
in case of climbing complex structures (like trees) or crawling across complex
terrain (such as passing wide openings or passing tall barriers).

### Individual performance

Single OpenVMP units can perform many tasks by themselves over an extended 
period of time.
OpenVPM hardware is designed to perform some tasks individually.
OpenVMP software provides support for keeping power usage at minimum while idle.

However the most important use case for individual performance is individual
OpenVMP units leaving the collective temporarily to perform an ad-hoc task
(recon, comms, supply, delivery etc).

## More information

See the following documents for more info:

- [ROS2 packages](docs/ROS2_packages.md)
- [Deployment](docs/Deployment.md)
- [Development](docs/Development.md)
- [Roadmap](docs/Roadmap.md)
- [License](docs/License.md)
