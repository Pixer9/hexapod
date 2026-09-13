# Whole-Body Kinematics Boundary

**Status:** Initial implementation baseline

## Purpose

The whole-body kinematics layer is the first bridge between future locomotion
planning and the canonical 18-joint command vector.

Input:

```text
six BODY-frame foot targets
```

Output:

```text
18 mathematical logical joint angles in degrees
```

The output order is fixed:

```text
RF coxa, RF femur, RF tibia,
RM coxa, RM femur, RM tibia,
RB coxa, RB femur, RB tibia,
LF coxa, LF femur, LF tibia,
LM coxa, LM femur, LM tibia,
LB coxa, LB femur, LB tibia
```

That order matches the established Pi/Servo 2040 logical-joint contract.

## Boundary ownership

This layer owns:

- BODY -> LEG coordinate transforms;
- six independent leg IK solves;
- canonical logical-vector assembly;
- deterministic rejection of malformed or unreachable foot-target sets.

This layer does not own:

- servo channel mapping;
- servo direction inversion;
- servo trim/calibration;
- hard physical servo limits;
- PWM generation;
- HX1 serialization;
- gait generation;
- controller input;
- command arbitration.

## Atomicity

A six-leg solve is atomic from the caller's perspective.

If any target set is malformed or any leg is unreachable, no partial
`RobotJointSolution` is returned.

`LegSolveError` identifies the failed leg and preserves the underlying
kinematics exception for diagnostics.

## Configuration

The solver consumes the already-loaded `RobotGeometry`. No additional physical
values are hard-coded into whole-body kinematics.

Robot variants therefore change geometry through:

```text
config/robots/<variant>.json
```

rather than by forking solver code.

## Reference stance

`RobotKinematics.reference_stance_body()` converts the configured
`neutral_foot_leg_mm` reference point into six BODY-frame foot positions.

This is a geometry/reference fixture. It is not the walking gait's stance
height policy. Runtime/gait stance parameters will remain separately
configurable.
