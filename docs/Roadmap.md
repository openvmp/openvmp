# OpenVMP

Below you can see the ideas that you can work on as a contributor.

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

## Ecosystem features

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
