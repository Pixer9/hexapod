# Pi Joint Planning Soft Limits

**Status:** Initial standard-robot baseline

## Purpose

The Raspberry Pi owns planning-level soft motion limits. The Servo 2040 remains
the authoritative physical actuator safety boundary.

The post-IK normal-motion path is:

```text
RobotKinematics
    |
    v
18 canonical logical joint angles
    |
    v
JointSoftLimitProfile.validate()
    |
    v
JointTrajectoryRateScaler
    |
    v
future HX1 client
    |
    v
Servo 2040 hard validation
```

The Pi soft-limit layer rejects a complete vector atomically. It never clamps
individual joints.

## Configuration

The standard robot planning profile is:

```text
config/robots/standard-joint-soft-limits.json
```

Initial planning ranges are:

| Joint type | Minimum | Maximum |
|---|---:|---:|
| Coxa | -75 deg | +75 deg |
| Femur | -90 deg | +38 deg |
| Tibia | +15 deg | +115 deg |

The values are intentionally conservative relative to the current Servo 2040
hard logical envelope.

They are robot-planning limits, not copies of physical servo calibration.

## Standard tripod baseline

The initial standard tripod swing height is reduced from the migrated 70 mm
value to:

```text
30 mm
```

The prior 70 mm value can drive the canonical femur solution above the current
40 degree Servo 2040 hard logical maximum.

A 30 mm baseline leaves planning margin beneath the 38 degree Pi femur soft
maximum during the configured command/phase regression sweep.

This value is an initial safe software baseline, not final gait tuning. Physical
walking tests may tune it later, but any new value must continue to satisfy the
Pi planning envelope and the Servo 2040 hard envelope.

## Cross-boundary compatibility

Tests derive the current Servo 2040 hard logical ranges from its physical
actuator profile and verify every Pi soft range is contained inside the
corresponding MCU hard range.

This is a development-time compatibility check only. Runtime HX1 negotiation
must still verify actuator profile identity before arming.

## Failure behavior

A soft-limit violation is a planning error.

The normal Pi pipeline must not:

- silently clamp a joint;
- independently alter offending joints;
- send the invalid vector and rely on the MCU to reject it.

The complete vector is rejected before the joint-rate scaler/HX1 boundary.

The Servo 2040 independently repeats authoritative hard validation because the
Pi is not trusted as the final hardware-safety authority.
