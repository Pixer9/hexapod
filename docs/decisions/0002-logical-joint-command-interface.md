# ADR-0002: Logical Joint Command Interface

**Status:** Accepted  
**Date:** 2026-09-12

## Context

The Pi and Servo 2040 require a stable motion interface.

Several possible command layers exist:

- raw PWM pulse widths;
- Servo 2040 physical channel angles;
- logical robot joint angles;
- foot positions;
- body velocity/gait commands.

The interface must preserve the ownership boundary established in ADR-0001.

The Pi should not need to know board-specific servo channels, while the MCU should not need to know robot gait or IK.

## Decision

The Pi will command the Servo 2040 using a complete vector of **logical joint angles**.

For the current standard Hexapod:

```text
18 logical joints
```

The wire representation is integer centidegrees.

Example:

```text
12.34 degrees  → 1234
-8.50 degrees  → -850
```

The actuator profile on the Servo 2040 maps each logical joint index to:

- physical servo channel;
- physical direction;
- zero/trim offset;
- hard minimum;
- hard maximum;
- hard maximum command rate;
- servo pulse characteristics when required.

The Pi may use floating-point math internally and converts to wire units at the MCU boundary.

## Logical Joint Ordering

Protocol messages carry logical joint indices rather than physical channel numbers.

The logical ordering is defined by the actuator profile and must remain stable for a given profile revision.

The Pi verifies the MCU actuator profile identity before arming.

Changing logical joint ordering requires an actuator-profile revision and coordinated Pi configuration update.

## Validation Model

The Pi applies planning-level soft limits.

The Servo 2040 independently applies hard physical limits.

Hard limits are authoritative.

An invalid target is **rejected**, not silently converted into a different valid target.

For a multi-joint target:

- validation is atomic;
- if one joint is invalid, the entire vector is rejected;
- the previous valid target remains active;
- no partial joint update occurs.

A final PWM layer may defensively bound values to its electrical/library range, but this must not make an invalid logical target appear successfully accepted.

## Rationale

Logical joint angles are the narrowest interface that keeps the correct knowledge on each side.

### Why not raw PWM?

Raw PWM would force the Pi to know physical servo calibration and would bypass MCU safety mapping.

### Why not physical Servo 2040 channel angles?

Physical channel angles still leak wiring/channel layout into the Pi and make robot variants harder.

### Why not foot XYZ coordinates?

Foot coordinates would require IK on the MCU, violating the Pi/MCU responsibility boundary.

### Why not velocity/gait commands?

That would require gait generation and robot behavior on the MCU, also violating the ownership boundary.

## Integer Wire Units

Integer centidegrees are used instead of floating-point text.

Benefits include:

- deterministic parsing;
- deterministic comparison;
- easy validation;
- no NaN or infinity representation;
- language-independent behavior;
- easier migration to packed/binary transport later if needed.

This choice does not require integer-only calculations inside the Pi.

## Commanded Versus Measured Position

The current hobby servos do not provide true position feedback.

Therefore the MCU knows:

```text
commanded logical joint position
```

It does not know:

```text
actual measured joint position
```

Telemetry, logs, APIs, and the dashboard must preserve that distinction.

No software layer may rename commanded servo state as actual/measured state without real feedback hardware.

## Consequences

### Positive

- Pi motion code is independent of Servo 2040 physical channel numbering.
- MCU safety remains authoritative.
- Robot variants can change wiring/calibration without rewriting gait code.
- Protocol payloads are compact and deterministic.
- A future MCU implementation language change does not alter robot-level semantics.

### Negative

- Pi and MCU profiles must agree on logical joint index semantics.
- Profile/version validation becomes mandatory.
- Servo calibration must be managed deliberately rather than hidden inside Pi motion code.

## Alternatives Considered

### Raw pulse width

Rejected because it bypasses MCU physical calibration ownership.

### Board-channel angle

Rejected because it exposes hardware wiring to high-level software.

### Cartesian foot target

Rejected because it moves IK ownership to the MCU.

### High-level velocity/gait command

Rejected because it moves locomotion planning to the MCU.

## Related

- [ADR-0001: Pi/MCU Responsibility Boundary](0001-pi-mcu-responsibility-boundary.md)
- [Pi–Servo 2040 Protocol v1](../protocols/pi-servo2040-v1.md)
- [Control Boundary](../architecture/control-boundary.md)
