# Pi Tripod Gait Trajectory

**Status:** Initial deterministic trajectory baseline

## Purpose

This layer converts the already-selected planar `MotionCommand` into six
BODY-frame foot targets. It sits between command arbitration/shaping and whole-
body inverse kinematics.

```text
MotionCommand
    |
    v
tripod gait trajectory
    |
    v
six BODY-frame foot targets
    |
    v
RobotKinematics
    |
    v
18 canonical logical joint angles
```

The gait layer does not read the DS4, arbitrate command sources, manage robot
lifecycle state, talk HX1, map servos, or perform hardware I/O.

## Configuration

The initial standard gait configuration is:

```text
config/locomotion/tripod.json
```

It migrates the useful v4 baseline values while giving them explicit semantics:

```text
cycle_hz              1.0
step_height_mm         70.0
stance_z_mm          -135.0
duty_factor             0.5
max_foot_offset_mm     50.0
```

`max_foot_offset_mm` is the maximum horizontal half-excursion of any foot from
its stance anchor. If the requested body twist would exceed that value, one
global scale factor is applied to all six leg sweeps so the requested twist
ratio is preserved rather than clipping legs independently.

## Tripod topology

The topology is a fixed semantic part of this gait, not variant geometry:

```text
Tripod A: RF RB LM
Tripod B: RM LF LB
```

Tripod B is one half-cycle out of phase with Tripod A.

With the baseline `duty_factor = 0.5`, one tripod is in stance while the other
is in swing. The loader rejects duty factors below 0.5 for this baseline so it
does not introduce a phase where neither tripod has support.

## BODY-frame twist to foot sweep

For a stance anchor `r = (x, y)`, the instantaneous BODY point velocity from a
planar body twist is approximated as:

```text
point_vx = vx - omega * y
point_vy = vy + omega * x
```

where `omega` is in radians/second.

The desired half-sweep is that point velocity multiplied by half the stance
time. During stance the foot moves from `+half_sweep` to `-half_sweep`, giving
an average BODY-relative foot motion opposite the requested body motion.

This is intentionally a first-order planar twist model around the configured
stance anchor. Exact SE(2) arc integration is not part of this first gait
baseline.

## Smooth trajectory shape

Horizontal stance and swing motion use quintic smootherstep:

```text
s(u) = 6u^5 - 15u^4 + 10u^3
```

It has zero first and second derivatives at both endpoints. That gives C2
position continuity across fixed-command stance/swing boundaries without the
horizontal overshoot that would be required by a constant-velocity stance
joined to a velocity-matched return swing.

Swing lift uses:

```text
lift(u) = 64 u^3 (1-u)^3
```

which is also zero in position, velocity, and acceleration at the swing
endpoints and reaches one at `u = 0.5`.

## Blend boundary

`TripodGait.sample()` accepts a caller-owned `blend` from 0 through 1. It scales
both horizontal excursion and foot lift.

```text
blend = 0 -> flat stance
blend = 1 -> full requested gait trajectory
```

A zero `MotionCommand` always returns the flat stance even when the caller
passes `blend=1`.

The gait layer deliberately does not decide how `blend` evolves with time. A
later locomotion/runtime layer will own start/stop transitions and the separate
command-rate limiter already planned ahead of gait generation. This keeps the
trajectory sampler deterministic and free of hidden clocks.

## Relationship to v4

V4 established useful starting dimensions and the same tripod membership, but
its implementation used global mutable leg latches, per-half-cycle chasing, and
smoothstep-based target construction tied directly to wall-clock time.

The new baseline keeps the proven geometry/tuning values as migration data but
replaces those runtime mechanics with a pure phase-driven sampler. Old v4 code
remains reference material only.

## Safety and reachability

The gait limit is not a servo hard limit. It is a Pi-side trajectory bound.
Servo 2040 hard joint limits remain authoritative later in the chain.

The test suite samples the configured full planar command across a complete
gait cycle and sends every generated six-foot target set through
`RobotKinematics`. Any unreachable leg therefore fails the Pi test suite before
hardware is involved.
