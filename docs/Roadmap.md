# OpenVMP

Below you can see some draft ideas that you can work on as a contributor.

Reach out to opevmp@proton.me to discuss more details and/or submit your pull request.

## Short term tactical needs

- rewrite the existing FreeCAD design using CadQuery
- sensor filter in 'remote_encoder' to ignore faulty readings
- add support for joint brakes to the joint trajectory controller
- add odometry components
- add vision components
- add SLAM components

## Long term strategic goals

### Robot hardware features

- Uplink modules
  - Template module to bring connectivity up and down based on run-level
  - 4G/LTE module
  - GPRS module
  - Satellite data module
  - 5G module
- Audio module (optional)
- NUC management extension (optional)
- Contactless charging from power lines (optional)
  - City power lines
  - Intra-city power lines
- Solar charging
  - Statically mounted (optional, with a limited surface)
  - Deployable (optional, with a larger surface)

### Robot software features

- Communications module
  - subscribe to events on DDS and push them to gRPC
  - subscribe to events on gRPC and push them to DDS
- Audio module (requires hardware)
  - Audio recording
  - Voice recognition (command match)
  - Voice recognition (person match)
- NUC management extension (requires hardware)
  - Module on Raspberry PI to bring NUC up and down gracefully
  - Module on Raspberry PI to program NUC
    - on first boot / firmware upgrade
    - OTA update from Cloud

### OpenVMP ecosystem features

- Cloud microservices & API (Golang + gRPC + Protobuf)
  - Location reporting (sub, robot to report)
  - Location monitoring (pub, clients to monitor)
- React-native-web application configurable to use arbitrary API URLs
- Cloud deployment scripts to deploy a private instance of OpenVMP API
  - CloudFormation for AWS
  - Terraform for GCP
  	 - AppEngine
  	 - Docker in GCE
  	 - GKE
  - Azure
- Parts database
  - specs
  - up-to-date links where they can be purchased
- VR
  - Robot control using Unity
  - Support for Oculus Quest 2
    - render from PC
    - render from cloud

### ROS2 ecosystem features

- Extensions for RViz interactive markers
  - to display play/pause symbols for each joint (in the center of the same plane where the other markers are displayed now) to be used for pausing movement routines or for engaging/disengaging joint brakes
