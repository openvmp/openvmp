# OpenVMP

Below you can see some draft ideas that you can work on as a contributor.

Send us a pull request with the suggested implementation.

## Robot hardware features

- Uplink modules
  - Template module to bring connectivity up and down based on run-level
  - 4G/LTE module
  - GPRS module
  - Satellite data module
  - 5G module
- Audio module (optional)
- NUC extension (optional)
- Contactless charging from power lines (optional)
  - City power lines
  - Intra-city power lines
- Solar charging
  - Statically mounted (optional, with a limited surface)
  - Deployable (optional, with a larger surface)

## Robot software features

- Communications module
  - subscribe to events on DDS and push them to gRPC
  - subscribe to events on gRPC and push them to DDS
- Audio module (requires hardware)
  - Audio recording
  - Voice recognition (command match)
  - Voice recognition (person match)
- NUC extension (requires hardware)
  - Module on Raspberry PI to bring NUC up and down gracefully
  - Image for NUC to host computation hungry modules
    - Module for building point clouds using stereo cameras
    - Module for building a world model using point clouds
  - Module on Raspberry PI to program NUC
    - on first boot / firmware upgrade
    - OTA update from Cloud

## OpenVMP ecosystem features

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

## ROS2 ecosystem features

- Extensions for RViz interactive markers
  - to display play/pause symbols for each joint (in the center of the same plane where the other markers are displayed now) to be used for pausing movement routines or for engaging/disengaging joint brakes

![ROS/ROS2 index package for OpenVMP module: Roadmap](https://www.google-analytics.com/collect?v=1&tid=UA-242596187-2&cid=555&aip=1&t=event&ec=github&ea=md&dp=%2FRoadmap.md&dt=OpenVMP%20Documentation)