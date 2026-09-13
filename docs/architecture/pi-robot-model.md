# Raspberry Pi Robot Model and Kinematics Boundary

**Status:** Initial implementation baseline

## Purpose

The Raspberry Pi owns the mathematical robot model, coordinate transforms,
kinematics, gait generation, trajectory generation, command arbitration, and
higher-level autonomy.

The Servo 2040 owns physical actuator mapping and hard actuator safety.

## Configuration rule

Physical values that may change between robot variants belong in configuration,
including:

- link lengths;
- body leg-mount locations;
- body leg-mount yaw angles;
- reference stance geometry.

Semantic contracts do not become runtime configuration merely for flexibility.
The following remain code/protocol invariants:

- body frame: +X forward, +Y left, +Z up;
- canonical leg order: RF, RM, RB, LF, LM, LB;
- canonical joint meaning: coxa yaw, femur pitch, tibia/knee with 0 degrees
  straight and positive bend;
- the Raspberry Pi does not apply servo channel mapping, direction inversion,
  trim, pulse calibration, or physical actuator hard limits.

Those physical actuator details belong to the Servo 2040 actuator profile.

## Standard geometry migration baseline

The initial standard profile preserves the known v4 geometry:

- coxa: 41 mm;
- femur: 116 mm;
- tibia: 183 mm;
- leg-mount radius: 105 mm;
- leg yaws:
  - RF -45 degrees
  - RM -90 degrees
  - RB -135 degrees
  - LF +45 degrees
  - LM +90 degrees
  - LB +135 degrees
- reference local-leg neutral foot: (200, 0, -115) mm.

The v4 runtime also used a walking stance near -135 mm Z. That is a locomotion
parameter rather than immutable robot geometry and will live in gait/runtime
configuration when that layer is implemented.

## Kinematics behavior

The initial IK equations preserve the proven v4 solution branch and mathematical
joint convention.

One deliberate behavior change is made: an unreachable foot target raises
`UnreachableTargetError`. The new runtime does not silently project an
unreachable request onto the workspace boundary.

Tiny floating-point overshoot in the cosine-law terms may still be clipped to
[-1, +1] only after geometric reachability has already been established.

## Scaling

Additional physical variants should add another configuration file, for example:

```text
config/robots/mini.json
config/robots/standard.json
config/robots/large.json
```

Core kinematics should not fork by robot size.
