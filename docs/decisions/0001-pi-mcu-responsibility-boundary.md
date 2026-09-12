# ADR-0001: Pi/MCU Responsibility Boundary

**Status:** Accepted  
**Date:** 2026-09-12

## Context

The Hexapod contains two programmable computers with very different roles:

- Raspberry Pi 5 running the main Python application;
- Pimoroni Servo 2040 based on the RP2040 microcontroller.

Earlier experimental firmware placed robot geometry, inverse kinematics, gait interpolation, calibration, communication, and direct servo control together on the Servo 2040.

Later firmware moved closer to a narrower actuator-controller model with arm/disarm, direct 18-joint targets, interpolation, and PWM output.

A durable architecture is required before rebuilding the software stack.

The system must also support:

- future autonomous navigation;
- a web dashboard;
- multiple command sources;
- possible future robot variants;
- hardware safety independent of high-level application correctness;
- resource-conscious operation on an SBC;
- a future path toward kit/product quality.

## Decision

The Raspberry Pi is the **robot computer**.

The Servo 2040 is the **actuator and hardware-safety controller**.

The governing rule is:

> **The Raspberry Pi decides what motion the robot should perform. The Servo 2040 decides whether actuator commands are safe to execute.**

### Raspberry Pi responsibilities

The Pi owns:

- robot geometry;
- FK/IK;
- gait generation;
- foot trajectories;
- body pose;
- controller interpretation;
- command arbitration;
- autonomy;
- localization;
- mapping/SLAM;
- path planning;
- sensor fusion;
- robot-state aggregation;
- API/dashboard integration.

### Servo 2040 responsibilities

The MCU owns:

- logical-joint to physical-channel mapping;
- physical direction/trim;
- hard actuator limits;
- hard command-rate limits;
- PWM generation;
- arm/disarm enforcement;
- runtime safety state machine;
- communications watchdog;
- motion watchdog;
- foot-contact sampling;
- hardware-level telemetry;
- optional raw IMU/ToF acquisition if later selected.

The Servo 2040 will not perform robot-level IK, gait generation, navigation, or autonomy.

## Rationale

This boundary provides:

- a single place for high-level motion reasoning;
- deterministic hardware protection close to the actuators;
- easier testing of robot algorithms without physical hardware;
- a clean future migration path for MCU firmware implementation;
- simpler support for standard, mini, and large robot variants;
- reduced coupling between robot behavior and board-specific channel wiring;
- a safe failure mode if the Pi process crashes or sends invalid commands.

The Servo 2040 remains useful even when the Pi software is wrong because the MCU independently enforces actuator constraints.

## Consequences

### Positive

- High-level algorithms can be developed and unit-tested on the Pi.
- MCU firmware remains small and auditable.
- Hardware-specific mapping is isolated from robot logic.
- Future command sources all use the same Pi motion pipeline.
- A future web dashboard cannot accidentally bypass locomotion and write servos directly.
- A future C++ rewrite of the MCU firmware does not require moving robot logic.

### Negative

- The Pi↔MCU protocol becomes a critical interface.
- Both sides must share a stable definition of logical joint ordering.
- Some calibration information exists conceptually on both sides, with different ownership.
- Robust watchdog and session handling must be implemented.

## Alternatives Considered

### Put IK and gait generation on the Servo 2040

Rejected.

This couples robot geometry and behavior to actuator firmware and makes future variants and high-level testing harder.

### Use the Servo 2040 only as a completely passive PWM expander

Rejected.

The MCU is positioned ideally to enforce hard actuator limits, watchdogs, and local safety. Discarding that capability would make Pi failures more dangerous.

### Add a separate Arduino-class safety controller

Not selected.

The Servo 2040 already provides an RP2040 MCU and direct actuator ownership. A third controller is unnecessary unless future requirements demonstrate a need.

## Related

- [Control Boundary](../architecture/control-boundary.md)
- [ADR-0002: Logical Joint Command Interface](0002-logical-joint-command-interface.md)
- [ADR-0003: Servo 2040 Runtime Safety](0003-servo2040-runtime-safety.md)
