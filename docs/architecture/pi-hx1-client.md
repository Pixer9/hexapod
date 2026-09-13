# Pi HX1 Client Core

**Status:** Transport-independent client core with raw-byte transport adapters

## Purpose

The HX1 client core is the Raspberry Pi boundary between the completed logical
joint-motion pipeline and the Servo 2040 runtime protocol.

The normal path is:

```text
MotionCommand
    |
    v
CommandRateLimiter
    |
    v
LocomotionController
    |
    v
TripodGait
    |
    v
RobotKinematics
    |
    v
JointSoftLimitProfile.validate()
    |
    v
JointTrajectoryRateScaler
    |
    v
HX1ClientCore
    |
    v
HX1Transport
    |
    +---- FakeHX1Transport
    |
    +---- SerialHX1Transport
    |
    v
Servo 2040
```

The HX1 client core itself contains **no serial I/O** and does not energize
hardware.

It implements:

- HX1 v1 frame encoding/parsing;
- CRC-16/CCITT-FALSE;
- bounded newline framing;
- uint32 sender sequencing and wrap;
- typed parsing of `INFO`, `ACK`, `NACK`, `EVENT`, and `STATUS`;
- degree-to-centidegree wire conversion;
- Pi-side command-frame construction;
- HELLO/INFO session negotiation;
- required protocol-minor verification;
- exact actuator-profile ID/revision/SHA-256 verification;
- expected joint-count verification;
- zero-session ESTOP construction.

Raw byte I/O is defined separately by the HX1 transport boundary.

## Configuration

The current Servo 2040 link policy lives in:

```text
config/hardware/servo2040.json
```

The configured device is the stable by-id path:

```text
/dev/serial/by-id/usb-MicroPython_Board_in_FS_mode_e661410403724132-if00
```

The Pi does not depend on `/dev/ttyACM0`.

Transport baseline:

```text
kind               usb_cdc_serial
baudrate           115200
write timeout      0.25 s
read timeout       0 s (fixed by non-blocking transport contract)
```

Protocol timing baseline:

```text
client minor           1
required server minor  1
heartbeat period       100 ms
target period           20 ms
joint count             18
```

The heartbeat and target periods are normal transmission cadences. MCU watchdog
timeouts remain authoritative safety behavior and are not owned by this client.

## Peer identity policy

The current expected actuator profile is:

```text
profile id       hexapod-standard-v1
profile revision 1
profile SHA-256  9B214ED005E58E77A528B1ACDAB1E49BAD7C5ABBE98F42B41719A580A2C49060
```

The SHA-256 is over the exact deployed `actuator-profile.json` bytes, matching
the MCU profile fingerprint contract.

Tests verify that the Pi configuration fingerprint matches the exact profile
file currently stored in the repository.

A future actuator-profile revision must intentionally update the Pi expectation
before the new profile is accepted.

## Session behavior

Calling:

```python
hello = client.hello()
```

builds:

```text
HELLO|CLIENT_MINOR|EXPECTED_PROFILE_ID
```

and immediately invalidates any old local session authority.

Only a matching `INFO` referring to that HELLO may establish a new session.

The client verifies:

- `SERVER_MINOR >= required_server_minor`;
- exact profile ID;
- exact profile revision;
- exact profile hash;
- exact joint count.

Any mismatch prevents local session establishment.

The client does not infer that a successful session means the robot is ready to
arm. Runtime state and physical profile qualification remain separate concerns.

## ESTOP behavior

HX1 ESTOP is deliberately available without a negotiated session.

Before negotiation or after local session loss:

```python
client.estop("controller_estop")
```

uses:

```text
SESSION_OR_ZERO = 00000000
```

as allowed by the protocol.

This keeps the emergency-stop command independent of normal session authority.

## Joint wire conversion

The Pi motion pipeline uses floating-point degrees.

HX1 v1 uses integer centidegrees.

At the hardware boundary:

```text
degrees -> nearest centidegree integer
```

Exact half-centidegree ties are rounded away from zero.

Examples:

```text
12.344 deg -> 1234
12.345 deg -> 1235
-8.504 deg -> -850
-8.505 deg -> -851
```

The complete 18-joint vector is converted in canonical logical order. No servo
channel mapping, physical direction, trim, or PWM conversion occurs on the Pi.

## STATUS truthfulness

Protocol minor 1 includes `COMMAND_VALID`.

When `COMMAND_VALID = 0`, the transmitted `J0...J17` zeros are placeholders.
The Pi parser exposes:

```text
commanded_joint_cd = None
```

rather than treating the placeholders as a meaningful pose.

When `COMMAND_VALID = 1`, the vector is exposed as commanded logical state.

It is never labeled measured/actual servo position.

## Deliberate non-goals of this layer

The client/transport layers do not yet own:

- a background reader/writer loop;
- reconnect policy;
- heartbeat scheduling;
- 50 Hz target scheduling;
- ACK timeout/retry policy;
- robot lifecycle orchestration;
- SafetySupervisor behavior;
- automatic ARM/START transitions;
- physical servo qualification.

The raw transport contract is documented separately in
`pi-hx1-transport.md`.

## Current physical-arm limitation

The current Servo 2040 actuator profile is structurally valid but still has
`max_rate_cd_s = null` for the joints.

Therefore it is intentionally **not arm-qualified**.

HX1 development, HELLO/INFO negotiation, parsing, and serial transport testing
can proceed, but physical ARM/walking must remain blocked until the actuator
profile is physically qualified and revised.
