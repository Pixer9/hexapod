# Pi–Servo 2040 Protocol v1

**Status:** Accepted for implementation  
**Protocol major:** 1  
**Protocol minor:** 1  
**Date:** 2026-09-12

> **rc5 wire-format amendment:** Beginning with Servo 2040 firmware
> `0.1.0-rc5`, Pi-to-MCU authority-bearing commands use the compact binary
> command representation defined by ADR 0009. MCU-to-Pi INFO, ACK, NACK,
> EVENT, and STATUS representations remain ASCII HX1. ASCII command examples
> in this document should therefore be read as logical/diagnostic notation;
> ADR 0009 is authoritative for the Pi-to-MCU wire representation.
>
> Unsolicited periodic STATUS emission is disabled by default in rc5.
> `GET_STATUS` continues to provide explicit status retrieval.

## 1. Purpose

This document defines the version 1 control and telemetry protocol between the Raspberry Pi 5 and the Pimoroni Servo 2040.

The protocol carries:

- session negotiation;
- runtime safety commands;
- logical joint targets;
- watchdog heartbeats;
- MCU state;
- hardware telemetry;
- fault and emergency-stop information.

The protocol does not carry high-level gait, IK, navigation, mapping, or autonomy commands.

## 2. Design Goals

Version 1 prioritizes:

- deterministic parsing;
- simple implementation in Python and MicroPython;
- human-readable diagnostics;
- explicit versioning;
- safe failure behavior;
- bounded frame size;
- transport independence;
- straightforward future migration to C++ firmware if desired.

USB CDC serial is the initial transport.

The protocol is intentionally not tied to `/dev/ttyACM0`. The Pi should open the Servo 2040 using its stable device identity.

Current hardware identity:

```text
/dev/serial/by-id/usb-MicroPython_Board_in_FS_mode_e661410403724132-if00
```

Higher-level software should eventually make the device path configurable rather than hard-coded.

## 3. Normative Language

The terms **MUST**, **MUST NOT**, **SHOULD**, **SHOULD NOT**, and **MAY** describe protocol requirements.

## 4. Transport

Initial transport:

```text
USB CDC serial
```

The protocol itself is line-oriented ASCII and may be transported over another reliable byte stream in the future.

Normal runtime traffic and interactive REPL use must not be mixed.

When the production protocol is active, firmware must not emit arbitrary diagnostic `print()` output onto the protocol stream.

## 5. Frame Format

Every frame has this form:

```text
HX1|SEQ|TYPE|FIELD|FIELD|...|CRC16\n
```

A frame with no payload fields has this form:

```text
HX1|SEQ|TYPE|CRC16\n
```

### 5.1 Prefix

`HX1` identifies protocol major version 1.

A receiver that does not support the major version MUST NOT execute state-changing commands from that frame.

### 5.2 Sequence Number

`SEQ` is an unsigned 32-bit decimal sequence number.

Each sender maintains its own sequence space.

Sequence numbers increment monotonically modulo `2^32`.

For continuous motion targets, the MCU accepts only a sequence number that is newer than the previously accepted target sequence using the standard half-range modulo rule.

Duplicate and stale targets are rejected and do not refresh the motion watchdog.

### 5.3 Message Type

`TYPE` is an uppercase ASCII identifier.

`TYPE` MUST NOT exceed 32 ASCII characters.

Examples:

```text
HELLO
INFO
HEARTBEAT
STAGE
ARM
START
TARGET
STOP
DISARM
ESTOP
CLEAR_FAULT
CLEAR_ESTOP
GET_STATUS
STATUS
ACK
NACK
EVENT
```

### 5.4 Fields

Fields are ASCII.

Unless a message explicitly defines otherwise:

- integer fields use decimal notation;
- hexadecimal fields use uppercase hexadecimal;
- IDs may contain letters, digits, `.`, `_`, `-`, and `:`;
- fields must not contain `|`, `\r`, or `\n`;
- surrounding whitespace is not permitted.

No escaping mechanism exists in protocol v1.

### 5.5 Line Ending

Frames terminate with:

```text
\n
```

A receiver MAY tolerate one `\r` immediately before `\n`.

### 5.6 Maximum Frame Size

Maximum complete frame size, including newline:

```text
512 bytes
```

If a receiver accumulates more than 512 bytes without a valid frame terminator, it MUST discard input until the next newline and record a framing error.

### 5.7 CRC

The final field is a four-digit uppercase hexadecimal CRC-16/CCITT-FALSE.

Parameters:

```text
Name:       CRC-16/CCITT-FALSE
Polynomial: 0x1021
Initial:    0xFFFF
RefIn:      false
RefOut:     false
XorOut:     0x0000
```

The CRC is computed over the ASCII bytes of the complete frame body before the final CRC separator.

Example body:

```text
HX1|42|STOP|A1B2C3D4
```

Full conceptual frame:

```text
HX1|42|STOP|A1B2C3D4|972B
```

The bytes included in the CRC calculation are exactly:

```text
HX1|42|STOP|A1B2C3D4
```

The final `|CRC16` and newline are not included.

A CRC-invalid frame MUST NOT execute any command.

A CRC-invalid or structurally malformed frame should normally be dropped silently and counted in protocol diagnostics because its sequence number and contents cannot be trusted.

## 6. Wire Units

Protocol v1 uses integer wire units.

| Quantity | Unit |
|---|---|
| Joint angle | centidegrees |
| Time | milliseconds |
| Distance | millimeters |
| Voltage | millivolts |
| Current | milliamps |
| Temperature | centi-degrees Celsius |
| Foot contacts | bitmask |

Examples:

```text
12.34 degrees  → 1234
-8.50 degrees  → -850
7.4 volts      → 7400 mV
```

The Pi may use floating-point values internally. Conversion occurs at the hardware boundary.

NaN and infinity are not representable on the wire.

## 7. Logical Joint Vector

A full target contains exactly the number of joints advertised by the MCU.

The standard robot currently has:

```text
JOINT_COUNT = 18
```

Joint indices are logical indices, not Servo 2040 physical channel numbers.

The actuator profile defines:

- logical index;
- logical joint name;
- physical channel;
- physical direction;
- trim/zero;
- hard limits;
- hard slew/rate limit.

The Pi and MCU must agree on actuator profile identity before arming.

## 8. Sessions

A runtime session prevents stale traffic from a previous host process or reconnect from being treated as current control authority.

The Pi creates a session by sending `HELLO`.

The Servo 2040 responds with `INFO` containing a 32-bit session ID.

The session ID is represented as eight uppercase hexadecimal digits.

Example:

```text
A1B2C3D4
```

The session ID is regenerated when a new `HELLO` successfully establishes a session.

All state-changing commands after `HELLO`, except `ESTOP`, must carry the current session ID.

A wrong-session command is rejected.

A new session invalidates:

- any previously staged target;
- target sequence history associated with the prior session;
- authorization to arm.

A session does not survive MCU reboot.

## 9. Version and Profile Negotiation

### 9.1 HELLO

Pi → MCU

Schema:

```text
HX1|SEQ|HELLO|CLIENT_MINOR|EXPECTED_PROFILE_ID|CRC16
```

Example payload meaning:

```text
CLIENT_MINOR        = 0
EXPECTED_PROFILE_ID = hexapod-standard-v1
```

`HELLO` does not energize hardware.

If the protocol major is supported, the MCU responds with `INFO`.

### 9.2 INFO

MCU → Pi

Schema:

```text
HX1|SEQ|INFO|REF_SEQ|SESSION|SERVER_MINOR|FW_VERSION|MCU_ID|PROFILE_ID|PROFILE_REV|PROFILE_HASH|JOINT_COUNT|CAPS|STATE|CRC16
```

Fields:

| Field | Meaning |
|---|---|
| `REF_SEQ` | Pi `HELLO` sequence number |
| `SESSION` | New 32-bit session ID as 8 hex digits |
| `SERVER_MINOR` | MCU-supported protocol minor version |
| `FW_VERSION` | Firmware semantic/version identifier |
| `MCU_ID` | Stable MCU identity |
| `PROFILE_ID` | Actuator profile identifier |
| `PROFILE_REV` | Integer actuator profile revision |
| `PROFILE_HASH` | Opaque profile fingerprint |
| `JOINT_COUNT` | Number of logical joints |
| `CAPS` | Capability bit field as 8 hex digits |
| `STATE` | Current MCU state |

The Pi MUST verify at least:

- protocol major compatibility;
- required minor features;
- expected profile ID;
- expected profile revision/hash policy;
- expected joint count.

A profile mismatch prevents arming.

### 9.3 Minor-Version Rule

Protocol major changes are incompatible.

Protocol minor changes are backward-compatible additions.

A peer may use an optional feature only when that feature is advertised through capability negotiation or otherwise defined as available for the negotiated minor version.

## 10. Capability Flags

`CAPS` is a 32-bit hexadecimal bit field.

Initial assignments:

| Bit | Value | Capability |
|---:|---:|---|
| 0 | `0x00000001` | Six foot-contact inputs available |
| 1 | `0x00000002` | IMU acquisition available on MCU |
| 2 | `0x00000004` | ToF acquisition available on MCU |
| 3 | `0x00000008` | Voltage telemetry available |
| 4 | `0x00000010` | Current telemetry available |
| 5 | `0x00000020` | MCU status LED available |
| 6 | `0x00000040` | Physical E-stop input available |
| 7-31 | — | Reserved |

Unknown capability bits must be ignored by protocol-v1 receivers.

## 11. Heartbeat

### 11.1 HEARTBEAT

Pi → MCU

Schema:

```text
HX1|SEQ|HEARTBEAT|SESSION|HOST_UPTIME_MS|CRC16
```

Initial expected transmission rate:

```text
10 Hz
```

Only a valid `HEARTBEAT` for the current session refreshes the link watchdog.

`TARGET` traffic does not substitute for heartbeat.

## 12. Initial Target Staging

### 12.1 STAGE

Pi → MCU

Schema:

```text
HX1|SEQ|STAGE|SESSION|J0|J1|...|J17|CRC16
```

`STAGE` is valid only in `DISARMED`.

It provides the fresh initial logical-joint vector required before arming.

The MCU validates the entire vector.

If any value is invalid:

- the entire frame is rejected;
- no staged target is changed;
- `NACK` is returned.

A staged target expires after:

```text
2000 ms
```

A staged target is invalidated by:

- new session;
- MCU reset;
- fault;
- E-stop;
- successful transition out of the associated control flow as defined by the state machine.

## 13. Arming and Runtime Control

### 13.1 ARM

Pi → MCU

Schema:

```text
HX1|SEQ|ARM|SESSION|CRC16
```

Valid only in `DISARMED`.

Requirements include:

- valid current session;
- compatible profile;
- no fault;
- no E-stop;
- fresh valid staged target.

On success:

```text
DISARMED → ARMED
```

PWM is enabled and the staged target becomes the held command.

The MCU responds with `ACK`.

### 13.2 START

Pi → MCU

Schema:

```text
HX1|SEQ|START|SESSION|CRC16
```

Valid only in `ARMED`.

The link heartbeat must be fresh.

On success:

```text
ARMED → ACTIVE
```

The motion watchdog begins.

The MCU responds with `ACK`.

### 13.3 TARGET

Pi → MCU

Schema:

```text
HX1|SEQ|TARGET|SESSION|PERIOD_MS|J0|J1|...|J17|CRC16
```

Initial nominal values:

```text
PERIOD_MS = 20
rate      = 50 Hz
```

`TARGET` is valid only in `ACTIVE`.

The MCU validates:

- current session;
- state;
- sequence freshness;
- argument count;
- each logical target against hard limits;
- hard slew/rate constraints.

The entire target is atomic.

If any joint is invalid:

- no joint from the frame is applied;
- the previous valid target remains;
- the target is rejected;
- the motion watchdog is not refreshed.

Valid `TARGET` frames are normally not individually acknowledged.

The MCU reports the last accepted target sequence in `STATUS`.

### 13.4 STOP

Pi → MCU

Schema:

```text
HX1|SEQ|STOP|SESSION|CRC16
```

Valid only in `ACTIVE`.

On success:

```text
ACTIVE → ARMED
```

The MCU holds the last valid applied target.

The MCU responds with `ACK`.

### 13.5 DISARM

Pi → MCU

Schema:

```text
HX1|SEQ|DISARM|SESSION|CRC16
```

Valid in:

- `ARMED`;
- `ACTIVE`.

On success:

- motion is terminated;
- PWM is disabled;
- staged/active movement authority is cleared;
- state becomes `DISARMED`.

The MCU responds with `ACK`.

## 14. Emergency Stop

### 14.1 ESTOP

Pi → MCU

Schema:

```text
HX1|SEQ|ESTOP|SESSION_OR_ZERO|REASON|CRC16
```

`ESTOP` is special.

For safety, a syntactically valid, CRC-valid `ESTOP` is honored even when the supplied session does not match the current session.

`SESSION_OR_ZERO` may be:

- current session; or
- `00000000`.

`REASON` is a compact identifier such as:

```text
operator
pi_safety
web_estop
controller_estop
```

On acceptance:

- PWM is disabled immediately;
- active motion is terminated;
- staged target is invalidated;
- state becomes `ESTOP`.

The MCU sends `ACK` when possible and also emits an `EVENT`.

### 14.2 CLEAR_ESTOP

Pi → MCU

Schema:

```text
HX1|SEQ|CLEAR_ESTOP|SESSION|CRC16
```

Valid only in `ESTOP`.

If a physical E-stop input exists, it must be released first.

Success always transitions to:

```text
DISARMED
```

It never re-arms automatically.

## 15. Fault Recovery

### 15.1 CLEAR_FAULT

Pi → MCU

Schema:

```text
HX1|SEQ|CLEAR_FAULT|SESSION|CRC16
```

Valid only in `FAULT`.

The MCU accepts the command only when the underlying fault condition is no longer active.

Success always transitions to:

```text
DISARMED
```

A fresh staged target and new explicit `ARM` are required.

## 16. Status

### 16.1 GET_STATUS

Pi → MCU

Schema:

```text
HX1|SEQ|GET_STATUS|SESSION|CRC16
```

The MCU replies with an immediate `STATUS`.

Periodic status telemetry may also be emitted without a request.

### 16.2 STATUS

MCU → Pi

Schema:

```text
HX1|SEQ|STATUS|SESSION|STATE|FAULT|LAST_TARGET_SEQ|TARGET_AGE_MS|HEARTBEAT_AGE_MS|FOOT_MASK|BUS_MV|BUS_MA|UPTIME_MS|COMMAND_VALID|J0|J1|...|J17|CRC16
```

Fields:

| Field | Meaning |
|---|---|
| `SESSION` | Current session, or `00000000` when none |
| `STATE` | Current runtime state |
| `FAULT` | Current fault code or `NONE` |
| `LAST_TARGET_SEQ` | Last accepted `TARGET` sequence, or `-1` |
| `TARGET_AGE_MS` | Age of last accepted target, or `-1` |
| `HEARTBEAT_AGE_MS` | Age of last valid heartbeat, or `-1` |
| `FOOT_MASK` | Contact bits; `0` if unsupported/unwired |
| `BUS_MV` | Millivolts, or `-1` if unsupported |
| `BUS_MA` | Milliamps, or `-1` if unsupported |
| `UPTIME_MS` | MCU monotonic uptime |
| `COMMAND_VALID` | `1` when `J0...J17` contain a meaningful commanded vector; `0` when they are placeholders |
| `J0...J17` | Current **commanded** logical joint vector |

When `COMMAND_VALID` is `0`, `J0...J17` are transmitted as zero and MUST NOT
be interpreted as a commanded pose.

`COMMAND_VALID` describes the validity of the reported vector only. It does not
indicate that PWM is enabled or that the vector has current movement authority.

The joint vector is commanded state, not measured joint position.

An implementation must not label these values as actual/measured servo angles.

Suggested periodic status rate:

```text
10 Hz
```

This rate is telemetry-only and does not control the servo update rate.

## 17. ACK and NACK

### 17.1 ACK

MCU → Pi

Schema:

```text
HX1|SEQ|ACK|REF_SEQ|COMMAND|CRC16
```

Used for state-changing or explicitly requested operations.

`TARGET` frames are not ACKed individually during normal streaming.

### 17.2 NACK

MCU → Pi

Schema:

```text
HX1|SEQ|NACK|REF_SEQ|COMMAND|ERROR|CRC16
```

Initial error identifiers:

```text
ERR_BAD_SESSION
ERR_BAD_STATE
ERR_BAD_ARG_COUNT
ERR_BAD_VALUE
ERR_LIMIT
ERR_RATE_LIMIT
ERR_PROFILE_MISMATCH
ERR_NOT_READY
ERR_SEQ
ERR_ESTOP
ERR_FAULT_ACTIVE
ERR_STAGE_EXPIRED
ERR_UNSUPPORTED
```

Malformed or CRC-invalid frames may be dropped without `NACK`.

## 18. Events

### 18.1 EVENT

MCU → Pi

Schema:

```text
HX1|SEQ|EVENT|EVENT_TYPE|DETAIL|CRC16
```

Initial event types may include:

```text
BOOT
STATE_CHANGE
FAULT
ESTOP
ESTOP_CLEARED
FAULT_CLEARED
PROFILE_ERROR
```

`DETAIL` must remain a compact protocol-safe token.

Human diagnostic prose does not belong on the runtime channel.

## 19. Watchdog Rules

### 19.1 Link Watchdog

Initial contract:

```text
HEARTBEAT_RATE_HZ = 10
LINK_TIMEOUT_MS    = 750
```

Enforced in:

- `ARMED`;
- `ACTIVE`.

Only valid `HEARTBEAT` frames refresh it.

### 19.2 Motion Watchdog

Initial contract:

```text
TARGET_RATE_HZ    = 50
TARGET_PERIOD_MS  = 20
MOTION_TIMEOUT_MS = 200
```

Enforced only in `ACTIVE`.

Only accepted `TARGET` frames refresh it.

### 19.3 Watchdog Fault Response

For watchdog faults:

1. reject further movement;
2. retain the last valid applied target;
3. enter `FAULT`;
4. hold for up to `500 ms`;
5. disable PWM if the fault remains.

Other safety-critical faults may disable PWM immediately.

## 20. Hard Limits

The MCU is the final actuator-safety authority.

The Pi may enforce soft limits, but those are not substitutes for MCU hard limits.

For every target:

- each joint must be within its hard angle envelope;
- each joint change must comply with the configured hard rate/slew envelope;
- the complete vector must pass validation before any portion is applied.

Protocol v1 does not silently clamp an unsafe target.

Unsafe targets are rejected.

A final low-level PWM mapping layer may defensively clamp output to the electrical/servo-library range, but such clamping does not convert an invalid logical command into a successful command.

## 21. Persistent Data

Permitted persistent data includes:

- actuator profile;
- calibration;
- firmware metadata;
- profile ID/revision/hash.

Persisted runtime motion targets are not part of protocol v1.

There is no runtime `SAVE` or `LOAD` motion command.

MCU reboot always returns to a non-energized state requiring a fresh session, fresh staged target, and explicit arm.

## 22. Commands Intentionally Excluded from v1 Runtime

The legacy controller included commands such as direct channel set, LED control, power control, and saved-state operations.

Protocol v1 intentionally excludes general runtime commands equivalent to:

```text
SETCH
LED
POW
SAVE
LOAD
NEUTRAL
```

Reasons:

- direct channel commands bypass logical-joint abstraction;
- LED behavior belongs to MCU state indication;
- power/PWM state belongs to the runtime state machine;
- persisted motion state is intentionally prohibited;
- neutral posture is a robot-level concept owned by the Pi.

A dedicated maintenance/calibration protocol or mode may be designed later.

It must not weaken the normal runtime safety contract.

## 23. Startup Sequence

Expected startup:

```text
Pi                                              Servo 2040
│                                                   │
│ ---------------- HELLO --------------------------> │
│ <---------------- INFO --------------------------- │
│                                                   │
│ verify protocol/profile                           │
│                                                   │
│ ---------------- STAGE --------------------------> │
│ <---------------- ACK ---------------------------- │
│                                                   │
│ ---------------- HEARTBEAT ----------------------> │
│ ---------------- ARM ----------------------------> │
│ <---------------- ACK ---------------------------- │
│                                                   │
│ ---------------- START --------------------------> │
│ <---------------- ACK ---------------------------- │
│                                                   │
│ ======== TARGET at approximately 50 Hz =========> │
│ ===== HEARTBEAT at approximately 10 Hz =========> │
│ <========== STATUS at approximately 10 Hz ======= │
```

Stopping motion while remaining energized:

```text
STOP
  ↓
ARMED
```

Fully releasing servo outputs:

```text
DISARM
  ↓
DISARMED
```

## 24. Parser Requirements

The MCU parser must:

- be non-blocking;
- use bounded receive storage;
- enforce the 512-byte frame limit;
- reject malformed field counts;
- reject illegal numeric values;
- verify CRC before executing commands;
- avoid dynamic behavior that can unboundedly consume memory;
- never execute a partially parsed command;
- preserve the last valid actuator target when a new target is rejected.

## 25. Security Scope

Protocol v1 is a local embedded-control protocol, not a network-authentication protocol.

The session ID protects against stale control traffic; it is not a cryptographic credential.

Network-facing authentication and authorization belong at the Pi application/API boundary.

The Servo 2040 must not be directly exposed as a network service.

## 26. Future Compatibility

Potential future additions include:

- IMU telemetry;
- ToF telemetry;
- richer foot-contact telemetry;
- voltage/current measurements;
- physical E-stop input;
- maintenance/calibration mode;
- binary transport encoding if a demonstrated need arises.

These additions should preserve protocol-v1 safety semantics wherever possible.

A packed/binary protocol must not be introduced merely for optimization without evidence that the human-readable protocol is a real bottleneck.

## 27. Change Control

This protocol is accepted for implementation as version `1.0`.

After implementation begins:

- incompatible frame or semantic changes require a new protocol major;
- backward-compatible additions increment the protocol minor;
- safety-semantic changes require review of the runtime state-machine documentation and ADRs;
- implementation tests must include malformed, stale, wrong-session, out-of-limit, and watchdog scenarios.

Related documents:

- [Control Boundary](../architecture/control-boundary.md)
- [Runtime State Machine](../architecture/runtime-state-machine.md)
- [ADR-0001](../decisions/0001-pi-mcu-responsibility-boundary.md)
- [ADR-0002](../decisions/0002-logical-joint-command-interface.md)
- [ADR-0003](../decisions/0003-servo2040-runtime-safety.md)
