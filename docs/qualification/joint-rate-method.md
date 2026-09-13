# Joint-Rate Qualification Method

**Status:** Active qualification method
**Date:** 2026-09-13

## Objective

Determine defensible values for the Servo 2040 actuator-profile field:

```text
max_rate_cd_s
```

without guessing.

The value is the MCU-enforced hard maximum logical joint-command rate.

It is a command-slew safety ceiling. It is not a measured physical servo-shaft
velocity.

## Qualification Layers

Joint-rate qualification has three complementary layers. They answer different
questions and should not be conflated.

### Layer 1 — Motion-envelope software baseline

Use a known motion model, controller envelope, gait settings, and trajectory
generation to estimate the logical joint velocities that normal robot motion
requires.

This layer answers:

> What joint velocities does the intended motion envelope request?

The current baseline tool is:

```text
tools/qualification/joint_rates/
```

The tool reports observed software rates and engineering candidates. Its output
is an input to qualification, not a qualified actuator limit by itself.

### Layer 2 — Physical actuator hard-rate qualification

Exercise representative installed actuators progressively while the robot is
mechanically supported.

A typical sequence is:

1. verify the selected physical channel and direction at low speed;
2. use a conservative single-joint range;
3. increase the commanded rate progressively;
4. verify actual command timing and rate calculations;
5. watch for binding, harsh motion, oscillation, or abnormal strain;
6. verify non-selected channels remain disabled;
7. repeat for each mechanically distinct joint class;
8. perform additional cross-leg sanity checks where appropriate.

This layer answers:

> What hard logical command-rate ceiling is reasonable for the installed
> actuator and mechanism while preserving a defensible safety boundary?

A successful Layer 2 qualification may establish an arm-qualified actuator
profile for controlled energized HIL and commissioning.

It does **not** by itself qualify the full robot for gait or loaded walking.

### Layer 3 — Integrated locomotion validation

After the production Pi locomotion generator exists, exercise the complete
motion stack through supported poses, transitions, controlled gait, and
eventually loaded walking.

This layer answers:

> Does the production motion generator remain compatible with the qualified
> actuator hard-rate ceiling across the supported robot envelope?

Integrated validation may demonstrate that the existing hard ceiling remains
appropriate or may trigger actuator-rate requalification.

## Measurement

For every logical joint and every adjacent target frame:

```text
joint_rate_deg_s = abs(current_deg - previous_deg) / dt
```

Protocol units are:

```text
joint_rate_cd_s = joint_rate_deg_s * 100
```

At the planned 50 Hz Pi target cadence:

```text
dt = 0.020 s
```

Physical qualification must use actual elapsed time rather than assuming the
nominal period.

The Servo 2040 hard-rate rule is equivalent to:

```text
abs(delta_cd) * 1000 <= max_rate_cd_s * dt_ms
```

where `dt_ms` is the actual elapsed time between accepted command references.

## Software Candidate Rule

The software-only baseline computes a candidate using:

```text
candidate = round_up(peak * headroom_multiplier)
```

The initial headroom multiplier is:

```text
1.5
```

and candidates are rounded upward to:

```text
1000 cd/s
```

This candidate is an engineering input, not an automatically approved hard
limit.

The multiplier may be changed only with documented rationale.

## Joint Families

Results are reported for:

```text
coxa
femur
tibia
```

as well as all 18 individual logical joints.

If the six legs show materially equivalent requirements and physical behavior,
the final profile may use one common rate across multiple joints or joint
families.

If mechanical differences require otherwise, rates may remain per-joint.

## Revision 3 Qualification

Actuator-profile revision 3 established:

```text
max_rate_cd_s = 25000
```

for all 18 joints.

Representative coxa, femur, and tibia installations were progressively tested
through `250 deg/s` under supported, unloaded bench conditions. Additional
cross-leg sanity checks were also performed.

The detailed evidence is recorded in:

- `docs/qualification/joint-rate-rev3.md`

Revision 3 is therefore arm-qualified for controlled HIL/commissioning.

Production locomotion, gait, and loaded walking validation remain separate
integration activities.

## Requalification Triggers

Joint-rate qualification must be reviewed and repeated when any of the
following change materially enough to affect safe command rates:

- control target rate;
- link geometry;
- gait algorithm;
- gait cycle frequency;
- step height;
- stance height;
- duty factor;
- maximum step;
- maximum translation velocity;
- maximum yaw velocity;
- command smoothing;
- gait start/stop transitions;
- body-pose control;
- stabilization behavior;
- robot variant;
- actuator type;
- servo power configuration;
- mechanical linkage or load characteristics.

Not every motion-envelope change automatically requires a different hard
actuator limit, but it does require confirming that legitimate production
demand remains below the qualified ceiling with appropriate margin.

## Acceptance

A hard `max_rate_cd_s` value may be promoted into an arm-qualified actuator
profile only after:

- the intended meaning and enforcement semantics are reproducible;
- representative installed actuator classes have been physically tested;
- the selected value remains below a credible actuator/mechanical capability;
- the test range and conditions are documented;
- channel isolation and safe disable behavior are verified during testing;
- the actuator profile revision is incremented;
- the exact profile fingerprint is recorded;
- host-side actuator/state-machine safety tests pass.

Integrated locomotion validation is then required before treating the robot as
qualified for normal gait or loaded walking.

A future profile with a missing or non-positive required rate remains
unqualified for arming.
