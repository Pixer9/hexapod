# Servo 2040 Actuator Profile

**Status:** Arm-qualified actuator profile for controlled HIL/commissioning; integrated locomotion validation remains pending
**Profile:** `hexapod-standard-v1`
**Profile revision:** 3
**Schema version:** 1
**Date:** 2026-09-13

## Purpose

The actuator profile is the Servo 2040's authoritative description of the physical actuator layer.

It maps the robot's canonical logical joints to Servo 2040 channels and defines the physical calibration and hard actuator safety envelope.

The profile does **not** contain robot geometry, gait parameters, inverse kinematics, body dimensions, controller settings, or navigation configuration.

Those are Pi-owned concerns.

## Canonical Logical Joint Convention

Protocol joint vectors use canonical mathematical robot-joint coordinates.

Canonical leg order:

```text
RF, RM, RB, LF, LM, LB
```

Canonical joint order within each leg:

```text
COXA, FEMUR, TIBIA
```

Therefore the 18-element protocol vector is:

| Index | Joint | Servo 2040 channel |
|---:|---|---:|
| 0 | `rf_coxa` | 12 |
| 1 | `rf_femur` | 13 |
| 2 | `rf_tibia` | 14 |
| 3 | `rm_coxa` | 6 |
| 4 | `rm_femur` | 7 |
| 5 | `rm_tibia` | 8 |
| 6 | `rb_coxa` | 0 |
| 7 | `rb_femur` | 1 |
| 8 | `rb_tibia` | 2 |
| 9 | `lf_coxa` | 15 |
| 10 | `lf_femur` | 16 |
| 11 | `lf_tibia` | 17 |
| 12 | `lm_coxa` | 9 |
| 13 | `lm_femur` | 10 |
| 14 | `lm_tibia` | 11 |
| 15 | `lb_coxa` | 3 |
| 16 | `lb_femur` | 4 |
| 17 | `lb_tibia` | 5 |

`channel` is the zero-based channel index used by `ServoCluster`. For example, channel `12` corresponds to the board's physical Servo 13 output.

### Canonical kinematic signs

The Pi uses the mathematical joint convention from the robot model:

- Coxa: yaw about +Z.
- Femur: angle in the radial/Z leg plane.
- Tibia: `0°` is straight; positive values increase knee bend.

The Pi sends these canonical logical angles directly to the MCU.

No servo-specific direction, trim, or physical-channel mapping belongs in Pi locomotion logic.

## Physical Mapping Equation

For each joint, the Servo 2040 converts logical joint angle to physical servo angle using:

```text
servo_cd = offset_cd + direction * logical_joint_cd
```

Where:

- `servo_cd` is physical servo command in centidegrees;
- `logical_joint_cd` is the canonical joint command in centidegrees;
- `offset_cd` is the physical servo zero/trim;
- `direction` is either `+1` or `-1`.

The physical servo command must satisfy:

```text
servo_min_cd <= servo_cd <= servo_max_cd
```

The MCU rejects a logical target that would violate this physical envelope.

## Migration from V4

V4 used two transformations:

```text
canonical IK joint
    ↓
global JointMap
    ↓
servo-oriented joint
    ↓
per-channel calibration
    ↓
physical servo angle
```

The V4 global joint map inverted the tibia before per-servo calibration.

The new architecture removes that intermediate servo-oriented joint coordinate system.

The equivalent transform is folded into each actuator profile entry, so the new runtime path is:

```text
canonical Pi joint
    ↓
per-joint actuator profile
    ↓
physical servo angle
```

For the current robot, the resulting profile direction pattern is:

```text
COXA  = -1
FEMUR = +1
TIBIA = +1
```

for all six legs.

This is intentional.

## Authoritative Physical Calibration

Revision 2 corrected four migrated tibia upper servo commands that exceeded the
default ServoCluster `ANGULAR` calibration domain of `-90 deg..+90 deg`. The
legacy Servo 2040 control path also explicitly treated servo commands as
`-90 deg..+90 deg`.

Revision 3 preserves the revision 2 channel mapping, direction, offsets, and
physical position limits. Revision 3 adds the physically qualified hard logical
command-rate ceiling described below.

Only the backend-unrepresentable upper endpoints were corrected in revision 2.
This reduced the affected derived logical tibia maxima while keeping every
current Pi soft limit inside the MCU hard envelope.

The current physical calibration is:

| Joint | Ch | Direction | Offset | Servo range | Derived logical range |
|---|---:|---:|---:|---:|---:|
| RF Coxa | 12 | -1 | -4° | -90°…90° | -94°…86° |
| RF Femur | 13 | +1 | 37° | -63°…77° | -100°…40° |
| RF Tibia | 14 | +1 | -29° | -19°…90° | 10°…119° |
| RM Coxa | 6 | -1 | 7° | -90°…90° | -83°…97° |
| RM Femur | 7 | +1 | 40° | -60°…80° | -100°…40° |
| RM Tibia | 8 | +1 | -35° | -25°…90° | 10°…125° |
| RB Coxa | 0 | -1 | 6° | -90°…90° | -84°…96° |
| RB Femur | 1 | +1 | 45° | -55°…85° | -100°…40° |
| RB Tibia | 2 | +1 | -42° | -32°…88° | 10°…130° |
| LF Coxa | 15 | -1 | 9° | -90°…90° | -81°…99° |
| LF Femur | 16 | +1 | 48° | -52°…88° | -100°…40° |
| LF Tibia | 17 | +1 | -40° | -30°…90° | 10°…130° |
| LM Coxa | 9 | -1 | 6° | -90°…90° | -84°…96° |
| LM Femur | 10 | +1 | 31° | -69°…71° | -100°…40° |
| LM Tibia | 11 | +1 | -39° | -29°…90° | 10°…129° |
| LB Coxa | 3 | -1 | 8° | -90°…90° | -82°…98° |
| LB Femur | 4 | +1 | 40° | -60°…80° | -100°…40° |
| LB Tibia | 5 | +1 | -35° | -25°…90° | 10°…125° |

The derived logical range is not stored in the JSON profile.

It is calculated by `profile.py` so physical calibration remains the single source of truth.

## Profile Fields

Top-level fields:

| Field | Meaning |
|---|---|
| `schema_version` | Actuator-profile schema version |
| `profile_id` | Stable profile family identifier |
| `profile_revision` | Revision of this physical profile |
| `joint_count` | Expected number of logical joints |
| `angle_unit` | Must be `centidegree` for schema v1 |
| `rate_unit` | Must be `centidegree_per_second` for schema v1 |
| `joints` | Ordered actuator definitions |

Per-joint fields:

| Field | Meaning |
|---|---|
| `index` | Canonical logical joint index |
| `name` | Stable canonical joint name |
| `channel` | Zero-based Servo 2040 output channel |
| `direction` | Physical direction, exactly `+1` or `-1` |
| `offset_cd` | Servo-space trim/zero in centidegrees |
| `servo_min_cd` | Hard minimum physical servo command |
| `servo_max_cd` | Hard maximum physical servo command |
| `max_rate_cd_s` | Hard maximum logical command rate |

## Hard Slew Rate

Revision 3 qualifies the hard logical command-rate ceiling as:

```text
max_rate_cd_s = 25000
```

for all 18 joints.

This is equivalent to:

```text
250 deg/s
```

The value is an MCU-enforced maximum logical command slew rate. It does not
represent measured servo shaft velocity and does not claim that the physical
servo tracks 250 deg/s under every possible load.

Qualification used progressive single-joint physical testing on representative
coxa, femur, and tibia installations through 250 deg/s, followed by additional
cross-leg sanity checks.

The qualification record is:

- `docs/qualification/joint-rate-rev3.md`

A missing or non-positive required hard-rate value still makes a profile
unqualified for arming.

Arm qualification establishes that the profile contains the physical safety
parameters required for controlled energized HIL and commissioning. It does not
by itself qualify the complete locomotion stack or loaded walking behavior.

## Derived Logical Limits

For a profile entry:

```text
servo = offset + direction * logical
```

the loader derives logical limits from the physical servo envelope.

For `direction = +1`:

```text
logical_min = servo_min - offset
logical_max = servo_max - offset
```

For `direction = -1`:

```text
logical_min = offset - servo_max
logical_max = offset - servo_min
```

All values remain integer centidegrees.

The loader must validate that the resulting logical range is non-empty.

## Profile Validation

The Servo 2040 self-test must reject a profile if any of the following are true:

- schema version is unsupported;
- profile ID is missing or invalid;
- profile revision is invalid;
- joint count is not 18 for this profile;
- joint array length does not match `joint_count`;
- indices are missing, duplicated, or out of order;
- canonical joint names are missing or duplicated;
- channels are missing, duplicated, or outside `0..17`;
- `direction` is not exactly `-1` or `+1`;
- servo minimum is not less than servo maximum;
- derived logical range is empty;
- required rate limit is missing or non-positive;
- any numeric field has an invalid type.

A profile-validation failure must prevent arming.

## Profile Fingerprint

The protocol `INFO` frame reports a profile fingerprint.

For profile schema v1:

```text
PROFILE_HASH = SHA-256(exact deployed actuator-profile.json bytes)
```

The JSON file does not contain its own hash.

This avoids a self-referential hash and ensures the fingerprint represents the exact deployed profile, including calibration values and ordering.

Deployment tooling must preserve the exact bytes whose hash is expected by the Pi.

The exact revision 3 fingerprint is:

```text
DF71DEBDB81A02999715B201DBEE7B7CF1363937E81C9D19449C4DBAA9178177
```

## Revision Rules

Increment `profile_revision` whenever a change can affect physical actuation or logical interpretation, including:

- channel mapping;
- direction;
- trim/offset;
- servo hard limit;
- hard slew/rate limit;
- logical joint ordering;
- joint naming semantics.

Formatting-only changes also change the SHA-256 fingerprint even when the revision does not change. Therefore deployment should normally avoid formatting-only profile edits.

## Related Documentation

- `docs/architecture/control-boundary.md`
- `docs/architecture/runtime-state-machine.md`
- `docs/protocols/pi-servo2040-v1.md`
- `docs/qualification/joint-rate-method.md`
- `docs/qualification/joint-rate-rev3.md`
- `docs/decisions/0002-logical-joint-command-interface.md`
- `docs/decisions/0004-actuator-profile-coordinate-model.md`
