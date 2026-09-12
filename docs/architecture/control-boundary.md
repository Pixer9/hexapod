# Hexapod Control Boundary

**Status:** Accepted for implementation  
**Version:** 1.0  
**Date:** 2026-09-12

## Purpose

This document defines the architectural boundary between the Raspberry Pi 5 and the Pimoroni Servo 2040.

The boundary is intentionally narrow:

> **The Raspberry Pi decides what motion the robot should perform. The Servo 2040 decides whether actuator commands are safe to execute.**

This rule is foundational. New features must preserve it unless a later Architecture Decision Record (ADR) explicitly supersedes it.

## System Roles

### Raspberry Pi 5: Robot Computer

The Raspberry Pi 5 is the robot's high-level computer. It owns behavior, motion intent, robot geometry, perception, navigation, and operator interfaces.

The Pi owns:

| Responsibility | Notes |
|---|---|
| Robot geometry and model | Body dimensions, leg origins, link lengths, coordinate frames |
| Forward kinematics | Robot-level computation |
| Inverse kinematics | Robot-level computation |
| Gait generation | Tripod and future gait strategies |
| Foot trajectory generation | Swing/stance trajectories and timing |
| Body pose control | Translation and orientation targets |
| Motion command arbitration | Selects the active command source |
| DS4 input | Primary manual controller during development |
| Future handheld controller | Must enter through the same command abstraction |
| Web control | Must enter through the same command abstraction |
| Autonomous control | Must enter through the same command abstraction |
| Perception | LiDAR and future perception inputs |
| Localization | Robot pose estimation |
| Mapping / SLAM | Runs on the Pi |
| Path planning | Global and local planning |
| Sensor fusion | Combines IMU, LiDAR, contact, and other measurements |
| Robot state model | Canonical state consumed by telemetry and interfaces |
| Telemetry aggregation | Hardware data plus robot-level state |
| Dashboard/API | Observer/control adapter; never a direct actuator driver |
| Soft motion limits | Planning limits used before commands reach the MCU |

The Pi produces logical joint targets. It does not produce raw PWM values.

### Servo 2040: Actuator and Hardware-Safety Controller

The Servo 2040 is the deterministic hardware boundary. It is not the robot's motion planner.

The Servo 2040 owns:

| Responsibility | Notes |
|---|---|
| Servo PWM generation | Direct control of all 18 servo outputs |
| Logical-joint to physical-channel mapping | Defined by the actuator profile |
| Physical servo direction | Hardware-specific mapping |
| Physical zero/trim calibration | Hardware-specific mapping |
| Hard actuator limits | Final protection against unsafe commands |
| Hard slew/rate limits | Final bound on actuator command rate |
| Arm/disarm enforcement | PWM state is controlled locally |
| Runtime state machine | BOOTING through ESTOP |
| Communications watchdog | Detects loss of the Pi control link |
| Motion freshness watchdog | Detects loss of fresh motion targets |
| Foot-contact sampling | Six foot switches when installed |
| Hardware telemetry | Contact state, faults, timing, power data when available |
| Status LED behavior | Represents MCU/runtime safety state |
| Optional raw sensor acquisition | IMU and ToF placement remains undecided |

The Servo 2040 must remain capable of protecting the hardware even if the Pi sends invalid data or the Pi application fails.

## Responsibility Matrix

| Capability | Pi 5 | Servo 2040 |
|---|:---:|:---:|
| Robot geometry | ✓ | |
| FK / IK | ✓ | |
| Gait generation | ✓ | |
| Foot trajectories | ✓ | |
| Body pose control | ✓ | |
| DS4 interpretation | ✓ | |
| Web control | ✓ | |
| Command arbitration | ✓ | |
| Autonomous navigation | ✓ | |
| Mapping / SLAM | ✓ | |
| Sensor fusion | ✓ | |
| Logical joint targets | ✓ | Receives |
| Servo channel mapping | | ✓ |
| Physical actuator calibration | | ✓ |
| Hard actuator limits | | ✓ |
| PWM generation | | ✓ |
| Hard slew/rate limits | | ✓ |
| Arm/disarm enforcement | | ✓ |
| Communications watchdog | | ✓ |
| Foot-contact acquisition | Consumes | ✓ |
| Hardware telemetry | Consumes | ✓ |
| LiDAR acquisition | ✓ | |
| IMU/ToF raw acquisition | TBD | TBD |

## Command Source Boundary

All high-level command sources must converge before locomotion.

The intended arbitration model is:

```text
Emergency stop
      ↓
Manual controller
      ↓
Web/manual control
      ↓
Autonomy
      ↓
Idle
```

The exact policy may evolve, but no command source may bypass the Pi motion pipeline and directly command the Servo 2040.

Every source ultimately produces the same robot-level command abstraction, such as:

```text
vx
vy
yaw_rate
body_pose
mode
```

The locomotion stack converts that command into trajectories and then into logical joint targets.

## Logical Versus Physical Joint Space

The Pi reasons in **logical joint space**.

A logical joint has:

- a canonical positive direction;
- a robot-level joint name/index;
- a desired angle;
- planning-level soft limits.

The Servo 2040 translates a logical joint target into a physical servo output:

```text
logical joint target
        ↓
logical joint index
        ↓
physical channel mapping
        ↓
direction
        ↓
zero / trim
        ↓
hard physical limits
        ↓
hard rate limit
        ↓
PWM output
```

The Pi must not depend on raw Servo 2040 channel numbers.

## Calibration Ownership

Calibration is divided into two categories.

### Robot calibration — Pi-owned

Examples:

- body dimensions;
- link lengths;
- leg origins;
- coordinate frame definitions;
- logical joint definitions;
- gait parameters;
- body pose parameters;
- planning-level soft limits.

### Actuator safety calibration — Servo 2040-owned

Examples:

- logical joint index to physical servo channel;
- physical direction;
- zero/trim offset;
- hard minimum and maximum angle;
- hard maximum command rate;
- servo pulse characteristics when required;
- foot-contact input mapping;
- hardware capability information.

The Pi may know a copy of actuator metadata for diagnostics, but the Servo 2040 is the final enforcement authority.

## Robot Variants

The architecture must support future robot variants without forking the control stack.

Examples may include:

```text
hexapod-standard
hexapod-mini
hexapod-large
```

A robot variant is represented by configuration profiles rather than a new codebase.

The Pi uses a robot profile. The Servo 2040 uses a corresponding actuator profile.

At connection time, the Pi verifies the actuator profile identity reported by the Servo 2040. A profile mismatch prevents arming.

## Sensors

### LiDAR

The RPLIDAR remains directly attached to the Pi.

LiDAR data is used by high-level perception, localization, mapping, and planning and therefore does not belong on the Servo 2040.

### Foot Contacts

The six leg contact switches should be sampled by the Servo 2040 when installed.

The MCU reports their state to the Pi as hardware telemetry. The Pi decides how contact information affects gait or state estimation.

### IMU and ToF

Physical ownership is intentionally not fixed in architecture version 1.0.

Two valid deployments remain possible:

1. IMU/ToF connected directly to the Pi.
2. IMU/ToF connected to the Servo 2040, with raw measurements forwarded to the Pi.

Regardless of wiring, sensor fusion remains on the Pi.

A later ADR will select the physical topology after the replacement IMU and driver behavior are known.

## Telemetry Truthfulness

The system must distinguish commanded values from measured values.

The current hobby servos do not provide joint-position feedback. Therefore:

- `commanded_joint_angle` is valid telemetry;
- `actual_joint_angle` is not valid unless real position feedback is added later.

The dashboard and APIs must never present commanded position as measured physical position.

## Non-Goals

The Servo 2040 does **not** own:

- gait selection;
- gait phase planning;
- foot trajectory generation;
- inverse or forward kinematics;
- body stabilization logic;
- path planning;
- obstacle avoidance;
- localization;
- mapping;
- SLAM;
- controller interpretation;
- autonomous decision-making.

These remain Pi responsibilities even if the MCU has enough computational capacity to perform some of them.

## Change Control

This document is an accepted architecture contract.

Changes to the Pi/MCU ownership boundary require:

1. a new or superseding ADR;
2. an explicit explanation of the safety and portability implications;
3. corresponding updates to the Pi–Servo 2040 protocol when applicable.

Related decisions:

- [ADR-0001: Pi/MCU Responsibility Boundary](../decisions/0001-pi-mcu-responsibility-boundary.md)
- [ADR-0002: Logical Joint Command Interface](../decisions/0002-logical-joint-command-interface.md)
- [ADR-0003: Servo 2040 Runtime Safety](../decisions/0003-servo2040-runtime-safety.md)
