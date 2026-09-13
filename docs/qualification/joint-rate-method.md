# Joint-Rate Qualification Method

**Status:** Initial software-baseline method  
**Date:** 2026-09-12

## Objective

Determine defensible values for the Servo 2040 actuator-profile field:

```text
max_rate_cd_s
```

without guessing.

The value is the MCU-enforced hard maximum logical joint-command rate.

## Qualification Stages

### Stage 1 — Legacy motion-envelope software baseline

Use the known-working V4 geometry, tripod gait behavior, gait settings, DS4 command envelope, and V4 command conditioning, but compute results in the new canonical logical-joint coordinates.

This stage answers:

> What joint velocities did the proven motion model require over the controller envelope?

Stage 1 is implemented by:

```text
tools/qualification/joint_rates/
```

### Stage 2 — Production-motion regression

After the new Pi locomotion stack exists, the qualification harness must exercise the production frame generator.

This stage answers:

> What joint velocities can the new software intentionally request over every supported runtime configuration?

The actuator hard limit must remain above legitimate production demand with deliberate engineering margin.

### Stage 3 — Physical validation

Candidates from software are tested progressively:

1. servo rail powered with robot secured;
2. conservative single-joint tests;
3. unloaded/low-load movement;
4. supported static poses;
5. controlled gait tests;
6. normal loaded walking.

Physical testing is used to confirm that the software candidate is reasonable for the actual DS3235 servos, linkage, power system, and robot mass.

## Measurement

For every logical joint and every adjacent Pi target frame:

```text
joint_rate_deg_s = abs(current_deg - previous_deg) / dt
```

Protocol units are:

```text
joint_rate_cd_s = joint_rate_deg_s * 100
```

At 50 Hz:

```text
dt = 0.020 s
```

The tool records each sample and reports peak, P99, and P99.9 rates.

## Candidate Rule

Stage 1 computes a candidate using:

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

This candidate is an engineering input, not an automatically approved hard limit.

The multiplier may be changed only with documented rationale.

## Joint Families

Results are reported for:

```text
coxa
femur
tibia
```

as well as all 18 individual logical joints.

If the six legs show materially equivalent requirements, the final profile may use one common rate per joint family.

If mechanical differences require otherwise, rates may remain per-joint.

## Requalification Triggers

Joint-rate qualification must be repeated when any of the following change materially:

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
- actuator type.

## Acceptance

A final `max_rate_cd_s` value is accepted only after:

- the software envelope is reproducible;
- the production locomotion generator is covered;
- physical validation succeeds;
- the selected rate remains below the actuator's credible physical capability;
- the actuator profile revision is incremented;
- qualification evidence is recorded.

Until then, `max_rate_cd_s` remains `null` and the new Servo 2040 firmware is not considered qualified to arm.
