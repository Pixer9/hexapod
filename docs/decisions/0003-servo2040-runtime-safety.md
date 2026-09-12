# ADR-0003: Servo 2040 Runtime Safety

**Status:** Accepted  
**Date:** 2026-09-12

## Context

The Hexapod has 18 powered servos and may eventually operate under manual, web, or autonomous control.

A high-level process failure must not leave the robot executing stale motion indefinitely.

The previous Servo 2040 firmware already supported arm/disarm behavior and direct joint commands, but the new architecture requires a stronger and explicit safety contract.

Important failure cases include:

- Pi process crash;
- USB link loss;
- locomotion loop hang while the rest of the Pi remains alive;
- stale buffered commands;
- invalid joint targets;
- wrong actuator profile;
- MCU reset;
- software-requested emergency stop;
- future physical E-stop input.

## Decision

The Servo 2040 will enforce a latched safety state machine with these states:

```text
BOOTING
SELF_TEST
DISARMED
ARMED
ACTIVE
FAULT
ESTOP
```

The MCU always boots with PWM disabled.

No persisted movement target may automatically authorize or cause motion after reboot.

## Explicit Arming

Arming requires all of the following:

- successful self-test;
- current valid protocol session;
- compatible protocol/profile;
- no fault;
- no E-stop;
- fresh complete staged joint vector;
- staged vector passing hard actuator validation.

The staged vector expires after 2000 ms.

Arming is always explicit.

Clearing a fault or E-stop never re-arms the robot.

## ARMED Versus ACTIVE

`ARMED` and `ACTIVE` are intentionally separate.

### ARMED

- PWM enabled;
- robot holds a validated command;
- heartbeat required;
- continuous motion targets not required.

### ACTIVE

- PWM enabled;
- heartbeat required;
- fresh target stream required;
- motion watchdog enabled.

`STOP` changes `ACTIVE` to `ARMED` and holds the last valid target.

`DISARM` disables PWM and returns to `DISARMED`.

## Independent Watchdogs

Two watchdogs are required.

### Link watchdog

Purpose:

Detect loss of the Pi supervisory process/link.

Initial values:

```text
heartbeat rate: 10 Hz
timeout:        750 ms
```

Only valid heartbeats refresh it.

### Motion watchdog

Purpose:

Detect a stopped locomotion loop even when the Pi remains alive enough to send heartbeats.

Initial values:

```text
target rate:  50 Hz
period:       20 ms
timeout:      200 ms
```

Only accepted motion targets refresh it.

Invalid targets do not refresh the motion watchdog.

## Fault Response

For watchdog faults, the MCU:

1. stops accepting further motion;
2. keeps the last valid command briefly;
3. enters `FAULT`;
4. holds for up to 500 ms;
5. disables PWM if the fault remains.

The brief hold is intended to reduce immediate uncontrolled collapse.

Safety-critical hardware faults may disable PWM immediately instead.

`FAULT` remains latched until an explicit `CLEAR_FAULT` succeeds after the underlying condition has cleared.

Successful clearing returns only to `DISARMED`.

## Emergency Stop

`ESTOP` is stronger than a normal stop.

On E-stop:

- PWM is disabled immediately;
- motion ends immediately;
- staged target is invalidated;
- state is latched as `ESTOP`.

A syntactically valid and CRC-valid E-stop command may be honored even if its session identifier does not match, because failing safe is preferred over ignoring a stop request.

Recovery requires explicit `CLEAR_ESTOP`.

If a physical E-stop is added later, it must also be physically released before clearing is accepted.

Successful clearing returns only to `DISARMED`.

## Invalid Motion Commands

Protocol-valid but unsafe motion is rejected atomically.

Examples:

- hard-limit violation;
- hard rate-limit violation;
- stale target sequence;
- wrong session;
- incorrect joint count.

The MCU:

- applies none of the invalid vector;
- keeps the last valid command;
- reports the rejection when possible;
- does not refresh the motion watchdog.

This design allows an isolated bad frame to be rejected without immediately collapsing the robot, while persistent invalid control naturally leads to a motion-timeout fault.

## Session Protection

A new host handshake creates a new MCU session identifier.

State-changing commands must normally match the active session.

This prevents stale buffered commands from an earlier host process or reconnect from being treated as current authority.

The session identifier is not a security credential. It is a stale-command safety mechanism.

## No Automatic Motion Restore

Persisted last-command state is explicitly rejected as a runtime safety mechanism.

After every MCU reset:

```text
PWM off
  ↓
self-test
  ↓
DISARMED
  ↓
fresh session
  ↓
fresh staged target
  ↓
explicit ARM
```

Persistent data is reserved for configuration and calibration.

## MCU-Enforced Hard Limits

The Pi may plan within soft limits, but the MCU is the final hard-limit authority.

The actuator profile must define enough information to prevent the Pi from directly commanding unsafe servo outputs.

Hard-limit enforcement remains active regardless of command source.

## Rationale

The Servo 2040 is physically closest to the actuators and continues running even when the Pi application misbehaves.

Therefore safety functions that protect against stale motion or invalid servo commands belong on the MCU.

Separating link health from motion freshness detects a wider set of failures than one generic watchdog.

Separating `ARMED` from `ACTIVE` allows the robot to hold a known pose without pretending that a motion stream is still live.

## Consequences

### Positive

- Pi crashes fail safe.
- A dead locomotion loop is detected even if the main process is still alive.
- Reconnects cannot casually reuse stale movement authority.
- Fault recovery always requires deliberate re-arming.
- Invalid commands are visible rather than silently clamped.
- The safety model can be tested independently of gait behavior.

### Negative

- Firmware and protocol logic are more complex than a simple serial servo bridge.
- Timeout values require hardware testing.
- Hobby servos without position feedback still cannot guarantee a gentle transition when PWM is first enabled.
- A 500 ms hold-before-release is a compromise and must be validated on the physical robot.

## Parameters Requiring Physical Validation

The following are accepted version 1 starting values, not immutable physical truths:

```text
target rate          50 Hz
motion timeout       200 ms
heartbeat rate       10 Hz
link timeout         750 ms
fault hold grace     500 ms
stage freshness      2000 ms
```

Changes require documented testing.

## Alternatives Considered

### One watchdog for all traffic

Rejected because a live heartbeat could hide a dead motion loop, while a motion stream alone could hide failure of supervisory logic.

### Immediately disable PWM on every bad target

Rejected because one malformed or out-of-limit target should not necessarily make the robot instantly collapse.

### Silently clamp unsafe targets

Rejected because it hides Pi-side defects and changes requested motion without making the failure visible.

### Restore the last commanded pose after reboot

Rejected because persisted motion must not become implicit authorization to energize hardware.

## Related

- [Runtime State Machine](../architecture/runtime-state-machine.md)
- [Pi–Servo 2040 Protocol v1](../protocols/pi-servo2040-v1.md)
- [ADR-0001: Pi/MCU Responsibility Boundary](0001-pi-mcu-responsibility-boundary.md)
- [ADR-0002: Logical Joint Command Interface](0002-logical-joint-command-interface.md)
