# ADR-0004: Actuator Profile and Joint Coordinate Model

**Status:** Accepted  
**Date:** 2026-09-12

## Context

The Hexapod needs a stable boundary between robot kinematics on the Raspberry Pi and physical servo calibration on the Servo 2040.

The legacy V4 control path mixed two transformations:

1. a global robot-joint-to-servo-oriented joint transform;
2. per-channel servo direction, offset, and hard physical limits.

V4's canonical robot model defined:

- Coxa as yaw about +Z;
- Femur in the leg radial/Z plane;
- Tibia as a knee angle where `0°` is straight and positive values increase bend.

V4 then applied a global tibia sign inversion before the per-channel servo calibration.

The new architecture requires the Pi to send canonical logical joint angles while the Servo 2040 owns all physical actuator mapping.

## Decision

The Pi-to-MCU protocol uses canonical mathematical robot joint angles.

The protocol vector order is permanently defined for `hexapod-standard-v1` as:

```text
RF.coxa
RF.femur
RF.tibia
RM.coxa
RM.femur
RM.tibia
RB.coxa
RB.femur
RB.tibia
LF.coxa
LF.femur
LF.tibia
LM.coxa
LM.femur
LM.tibia
LB.coxa
LB.femur
LB.tibia
```

The Servo 2040 actuator profile owns:

- logical index;
- canonical joint name;
- physical Servo 2040 channel;
- physical direction;
- physical offset/trim;
- hard physical servo minimum;
- hard physical servo maximum;
- hard maximum command rate.

The mapping equation is:

```text
servo_cd = offset_cd + direction * logical_joint_cd
```

V4's global tibia inversion is folded into the new per-joint actuator direction.

For the current robot this produces:

```text
COXA  direction = -1
FEMUR direction = +1
TIBIA direction = +1
```

for all six legs.

## Physical Versus Logical Limits

The profile stores physical servo-space limits:

```text
servo_min_cd
servo_max_cd
```

It does not duplicate derived logical limits.

The Servo 2040 derives the legal logical range from direction, offset, and the physical servo envelope.

This prevents logical and physical safety limits from becoming inconsistent copies of the same calibration.

## Canonical Joint Ordering

The existing V4 logical order is retained:

```text
RF, RM, RB, LF, LM, LB
```

with:

```text
COXA, FEMUR, TIBIA
```

inside each leg.

Physical channel numbering is deliberately independent of logical vector ordering.

Pi locomotion code must not depend on physical channel numbers.

## Wire Units

Actuator calibration uses integer centidegrees and centidegrees per second to match protocol v1.

No floating-point calibration is required on the wire or in the stored actuator profile.

## Hard Slew Qualification

The legacy implementation did not define a proven MCU-enforced hard actuator rate.

Therefore the initial migrated actuator profile stores:

```json
"max_rate_cd_s": null
```

for every joint.

`null` explicitly means the profile is incomplete for safe arming.

The Servo 2040 profile validator must prevent arming until every required rate has been physically qualified and populated.

After qualification, the profile revision must be incremented.

## Profile Identity

The profile has two identity mechanisms:

```text
profile_id
profile_revision
```

and an exact-byte fingerprint:

```text
SHA-256(actuator-profile.json)
```

The fingerprint is not stored inside the JSON file.

The Servo 2040 reports the profile identity and fingerprint during protocol negotiation.

The Pi verifies them before allowing an arm request.

## Rationale

This model keeps each concern in the correct layer:

```text
Pi
    mathematical robot joint
        ↓
Servo 2040
    physical actuator transform
        ↓
servo hardware
```

Benefits include:

- robot kinematics remain independent of board wiring;
- a replacement/rebuilt actuator assembly can change calibration without changing gait code;
- physical limits remain enforceable even when Pi software is incorrect;
- one canonical coordinate convention is used across IK, FK, telemetry, tests, and the protocol;
- the legacy tibia sign inversion no longer leaks into the Pi/MCU interface.

## Consequences

### Positive

- Cleaner mathematical model.
- No physical Servo 2040 channel numbers in Pi locomotion.
- No duplicate logical-limit configuration.
- Easier robot-variant support.
- Easier unit testing of both robot math and actuator mapping.
- MCU remains the final hardware-safety authority.

### Negative

- Migrated V4 calibration cannot be copied mechanically; tibia direction must reflect the removed global transform.
- Profile validation becomes safety-critical.
- Profile revision/fingerprint management must be disciplined.
- Hard slew rates must be qualified before the new firmware is allowed to arm.

## Alternatives Considered

### Preserve V4's global JointMap on the Pi

Rejected.

It exposes servo-oriented coordinate behavior above the actuator boundary and makes canonical robot joint telemetry ambiguous.

### Store both physical and logical hard limits

Rejected.

The two representations can drift. Logical limits are deterministic derivatives of the authoritative physical calibration.

### Use physical channel order as protocol vector order

Rejected.

This couples Pi locomotion to wiring and makes hardware revisions unnecessarily invasive.

### Invent a default hard slew rate

Rejected.

The legacy system provides no verified MCU hard-rate value. A safety parameter should not be created from an undocumented assumption.

## Related

- `docs/architecture/actuator-profile.md`
- `docs/architecture/control-boundary.md`
- `docs/protocols/pi-servo2040-v1.md`
- `docs/decisions/0001-pi-mcu-responsibility-boundary.md`
- `docs/decisions/0002-logical-joint-command-interface.md`
- `docs/decisions/0003-servo2040-runtime-safety.md`
