# OpenVMP

This document keeps track of the design considerations that are behind Open Versatile Mobility Platform.

## Contents

1. High Level Requirements
2. Detailed Design
3. Implementation Details

## High Level Requirements

### Mobility Modes

TBD

### Stepper Motors

Stepper motors and gearboxes should be as small, lightweight and power-efficient as possible to perform the task.

Stepper motors should be strong enough to hold the robot secure in any position if the additional 1g force is applied to the robot body in any direction while it's powered off.

TBD

### Servo Motors



## Detailed Design

### Stepper Motors

All stepper motors are in the holding position most of the time even when powered on.
So they will come with electromagnetic brakes to conserve power.

Electromagnetics brakes are also helpful to increase the torque when the robot is powered off.
However that's not going to be sufficient to hold the robot in place when it's not able to actively respond to the changing environment.
Therefore the stepper motors will always be connected through a gearbox.
The gearboxes will be of the worm type where possible to multiply stall torque.
Gearbox and motor sizing will be performed with the idea that the fastest required motion can be achieved at the top rpm that the motor supports,
while keeping the overall weight of the two as small as possible.

The stepper motors that are operating within a constrained angle will be of the open type with a separate absolute encoder.
The stepper motors that turn wheels will be of the closed type.

TBD

## Implementation Details

### Stepper Motors

NEMA 17 form factor will be used.
