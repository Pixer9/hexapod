# Pi Motion Pipeline

**Status:** Milestone 1 deterministic integration baseline

## Purpose

`MotionPipeline` is the hardware-independent composition layer that connects the
existing normal-motion stages into one deterministic update path:

```text
selected MotionCommand
    -> CommandRateLimiter
    -> LocomotionController / TripodGait
    -> RobotKinematics
    -> JointSoftLimitProfile
    -> JointTrajectoryRateScaler
    -> canonical 18-joint output
```

The pipeline owns no wall clock, source arbitration, HX1 session, heartbeat
scheduler, target scheduler, robot lifecycle authority, or hardware I/O.

Those remain responsibilities of the later runtime layer.

## Flat-stance initialization

The pipeline must be initialized before normal updates:

```text
initialize_flat_stance()
```

Initialization resets normal command/gait state and computes the configured gait
flat stance. For the current standard robot that stance uses:

```text
stance_z_mm = -135
```

This is intentionally different from the robot geometry reference foot Z of
`-115 mm`.

The initialization path is:

```text
configured gait flat stance
    -> whole-body IK
    -> Pi soft-limit validation
    -> joint trajectory scaler seed
```

The returned canonical 18-joint vector is the exact vector that the future
runtime must send in `STAGE` before `ARM`.

That keeps the Pi trajectory seed and the Servo 2040 staged command identical
and avoids an artificial startup jump between the geometry reference pose and
the walking stance.

## Update contract

After initialization:

```python
frame = pipeline.step(command, dt_s)
```

advances all normal-motion stages by the same explicit `dt_s`.

The final:

```text
frame.output_joint_vector_deg
```

is the canonical logical joint vector intended for later HX1 `TARGET`
serialization.

## Failure behavior

The pipeline contains several stateful normal-motion components.

If a downstream solve or validation fails after an upstream component has
already advanced, continuing from that partially advanced state would make
recovery ambiguous.

Therefore any update exception latches the pipeline failed. Further updates are
rejected until explicit flat-stance reinitialization.

The future runtime must treat such a failure as a lifecycle/safety event rather
than attempting to continue normal target streaming.

## Safety boundary

This layer does not replace the Servo 2040 safety boundary.

The Pi still performs planning soft-limit validation and normal joint-rate
scaling before producing an output. The Servo 2040 remains authoritative for:

- hard actuator position limits;
- hard actuator command-rate limits;
- PWM ownership;
- lifecycle enforcement;
- heartbeat and target watchdogs;
- fault and E-stop behavior.

## Milestone acceptance

This milestone is complete when host tests demonstrate that:

- the pipeline cannot run before explicit flat-stance initialization;
- initialization uses the configured gait stance rather than the geometry
  reference Z;
- the exact staged vector also seeds the joint trajectory scaler;
- the full configured command envelope can run through command shaping,
  locomotion, IK, soft limits, and joint-rate scaling at 50 Hz;
- a normal stop returns to the exact initialized flat stance;
- an update failure latches the pipeline until explicit reinitialization.
