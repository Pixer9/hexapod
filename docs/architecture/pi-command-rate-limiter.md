# Pi Command-Rate Limiter

**Status:** Initial deterministic normal-command shaping baseline

## Purpose

The command-rate limiter sits after normal source arbitration and before the locomotion controller:

```text
DS4 / web / autonomy
        |
        v
CommandArbiter
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
future HX1 client
        |
        v
Servo 2040 safety boundary
```

The limiter shapes how quickly normal requested BODY-frame velocity can change.

It does not:

- arbitrate command sources;
- manage ESTOP or FAULT state;
- advance gait phase;
- generate foot trajectories;
- solve inverse kinematics;
- enforce joint-position soft limits;
- enforce joint-trajectory rate limits;
- communicate over HX1;
- enforce actuator hard limits.

Those responsibilities belong to other layers.

## Configuration

The standard command-rate baseline lives in:

```text
config/control/rate-limit.json
```

Initial values:

```text
translation acceleration   260 mm/s^2
translation deceleration   520 mm/s^2
yaw acceleration           280 deg/s^2
yaw deceleration           700 deg/s^2
```

These values are migration and tuning baselines derived from useful behavior in the v4 controller.

They are not qualified actuator safety limits.

The normal command envelope remains defined separately in:

```text
config/control/motion.json
```

The command-rate limiter validates both requested targets and produced outputs against that envelope.

## Explicit Deterministic Time

The public update boundary is:

```python
shaped = limiter.step(target, dt_s)
```

The limiter owns state but does not own a wall clock.

It does not:

- call `time.monotonic()`;
- sleep;
- invent elapsed time;
- clamp a large elapsed interval to a hidden maximum.

The caller supplies `dt_s`.

Negative or non-finite delta times are rejected.

A zero delta time is valid:

```text
dt_s = 0
```

and produces no movement.

This keeps limiter behavior deterministic for a given sequence of targets and elapsed-time values.

## Translation Is One Vector

`vx` and `vy` are not slew-limited independently.

Translation is treated as one BODY-frame velocity vector:

```text
v = (vx, vy)
```

Changes are measured in Euclidean command-velocity space.

For example, from rest toward:

```text
(80, 60)
```

the target magnitude is:

```text
100 mm/s
```

At the configured translation acceleration of:

```text
260 mm/s^2
```

a `0.1 s` update permits a velocity-vector change of:

```text
26 mm/s
```

The resulting command is therefore:

```text
(20.8, 15.6)
```

which preserves the target's `4:3` direction exactly.

This avoids arbitrary X/Y axis bias and prevents one translational axis from reaching its requested value before the other merely because commands are represented with two scalar components.

## Translation Acceleration and Deceleration

Translation remains one two-dimensional velocity vector throughout the transition.

Several cases have explicit behavior.

### Motion from rest

If the current translation vector is zero, the limiter moves directly toward the target using the configured translation acceleration rate.

```text
current = (0, 0)
target  = non-zero
```

### Motion toward zero

If the target translation vector is zero, the limiter removes the current velocity using the configured translation deceleration rate while preserving its direction until zero is reached.

```text
current = non-zero
target  = (0, 0)
```

### True direction reversal

If the current and target vectors point more than 90 degrees apart:

```text
current dot target < 0
```

the transition is handled as a reversal.

Reversal behavior is described separately below.

### Non-reversing transition

For all other non-zero, non-reversing translations, the limiter follows the straight command-space segment from the current vector to the target vector.

Define:

```text
delta = target - current
```

The segment is classified using:

```text
target_delta_projection = target dot delta
```

If:

```text
target_delta_projection < 0
```

the complete segment uses the configured translation deceleration rate.

Otherwise, the complete segment uses the configured translation acceleration rate.

For ordinary collinear motion in the same direction, this reduces to the expected behavior:

```text
80 -> 40   uses deceleration
40 -> 80   uses acceleration
```

The projection rule is used instead of repeatedly comparing current and target speed magnitudes because the latter can change classification partway through a two-dimensional turn.

## Timestep Partitioning

For a fixed target, equivalent elapsed time should produce the same translational command-space result whether that time is supplied as one update or several smaller updates, subject only to floating-point precision.

Consider:

```text
current = (80, 0)
target  = (0, 60)
```

The requested command-space segment is:

```text
delta = (-80, 60)
```

with magnitude:

```text
100 mm/s
```

The segment classification is:

```text
target dot delta
= (0 * -80) + (60 * 60)
= 3600
```

which is positive, so the complete segment uses the configured translation acceleration rate:

```text
260 mm/s^2
```

After `0.20 s`, the limiter may traverse:

```text
260 * 0.20 = 52 mm/s
```

which is `52%` of the complete 100 mm/s segment:

```text
(80, 0) + 0.52 * (-80, 60)
= (38.4, 31.2)
```

Therefore these two update patterns produce the same result:

```python
step(target, 0.20)
```

and:

```python
step(target, 0.10)
step(target, 0.10)
```

The reason is that, while following the same straight segment toward a fixed target, the remaining delta is only multiplied by a non-negative scalar:

```text
delta_remaining = k * delta_original
```

where:

```text
0 <= k <= 1
```

Therefore:

```text
target dot delta_remaining
= k * (target dot delta_original)
```

and its sign cannot change midway through the transition.

This property prevents normal Raspberry Pi scheduler jitter or different loop partitions from changing a turn from acceleration behavior to deceleration behavior, or vice versa.

## Direction Reversal

A translation target more than 90 degrees away from the current velocity is treated as a true reversal:

```text
current dot target < 0
```

The limiter does not rotate the existing velocity vector directly through the origin using the acceleration rate.

Instead, it performs two stages:

```text
current velocity
        |
        | deceleration rate
        v
      zero
        |
        | acceleration rate
        v
new target direction
```

The current vector is first reduced to zero using the configured translation deceleration rate.

If the supplied `dt_s` contains time remaining after zero is reached, the remaining time is immediately consumed accelerating toward the new target.

For example:

```text
current = +80 mm/s
target  = -80 mm/s
```

With:

```text
translation deceleration = 520 mm/s^2
translation acceleration = 260 mm/s^2
dt_s                     = 0.20 s
```

the time required to reach zero is:

```text
80 / 520
= 0.153846... s
```

leaving approximately:

```text
0.046154 s
```

for acceleration in the opposite direction.

That produces approximately:

```text
-12 mm/s
```

during the same update.

This reversal behavior is also timestep-partition deterministic.

## Yaw Rate Limiting

Yaw is a scalar command rather than a two-dimensional vector.

Its behavior follows the same general acceleration/deceleration policy:

```text
current yaw = 0
target yaw  != 0
    -> acceleration rate

target yaw = 0
    -> deceleration rate

same-sign target with smaller magnitude
    -> deceleration rate

same-sign target with equal or larger magnitude
    -> acceleration rate
```

A yaw sign reversal is handled in two stages:

```text
current yaw rate
        |
        | yaw deceleration
        v
      zero
        |
        | yaw acceleration
        v
opposite yaw rate
```

Any time remaining after reaching zero is consumed accelerating in the new direction.

## Reset Boundary

`reset()` is an explicit hard state change:

```python
limiter.reset()
```

sets the limiter output immediately to zero.

A supplied command may also be used:

```python
limiter.reset(command)
```

which immediately sets the limiter state to that validated `MotionCommand`.

Normal command flow should use:

```python
step()
```

rather than `reset()`.

The reset boundary exists so a future runtime or safety layer can explicitly discard normal limiter history when required.

The limiter itself does not decide when ESTOP, FAULT, DISARM, or other safety events require that behavior.

## Relationship to Command Arbitration

The command-rate limiter operates only after the normal command source has already been selected.

The normal source priority is handled elsewhere:

```text
DS4
 |
 v
WEB
 |
 v
AUTONOMY
 |
 v
IDLE
```

The limiter does not know which source produced the selected command.

A transition caused by source arbitration is therefore shaped through the same normal motion boundary as any other command change.

Emergency-stop behavior remains outside normal arbitration and outside this limiter.

## Relationship to Gait Blending

Command-rate limiting and gait blending are separate mechanisms.

The command-rate limiter controls how quickly requested BODY velocity changes:

```text
vx
vy
yaw_rate
```

The locomotion controller separately owns:

- gait phase advancement;
- normal start blending;
- normal stop blending;
- transition between `IDLE`, `STARTING`, `MOVING`, and `STOPPING`.

There should be exactly one normal BODY-command rate limiter in the Pi motion pipeline.

Additional hidden low-pass filters or independent per-axis slew stages should not be added downstream.

## Relationship to Joint Soft Limits

The command-rate limiter operates before gait generation and inverse kinematics.

It therefore cannot determine whether a resulting joint configuration fits the robot's planning envelope.

That responsibility belongs to the post-IK Pi soft-limit layer:

```text
RobotKinematics
        |
        v
JointSoftLimitProfile.validate()
```

A joint-position soft-limit violation rejects the complete logical joint vector.

The command-rate limiter does not substitute for that validation.

## Relationship to Joint-Trajectory Rate Scaling

BODY-command rate limiting and joint-rate limiting solve different problems.

This layer constrains changes in:

```text
vx
vy
yaw_rate
```

before locomotion.

After gait generation and inverse kinematics, the resulting 18-joint logical vector passes through:

```text
JointTrajectoryRateScaler
```

That layer globally scales the complete joint-space update when any canonical logical joint would exceed the configured Pi-side joint-rate design budget.

The two mechanisms are deliberately separate:

```text
BODY command continuity
    -> CommandRateLimiter

joint-space continuity
    -> JointTrajectoryRateScaler
```

The joint-trajectory scaler does not replace this command limiter, and this command limiter does not replace joint-trajectory scaling.

## Relationship to Servo 2040 Safety

This Pi-side limiter improves normal command continuity but is not an actuator safety boundary.

The Servo 2040 remains authoritative for:

- logical-joint to physical-channel mapping;
- physical servo direction;
- actuator zero and trim;
- hard physical position limits;
- qualified hard slew/rate limits;
- PWM generation;
- communications watchdog behavior;
- motion freshness watchdog behavior;
- arm/disarm enforcement;
- ESTOP and FAULT enforcement.

The complete normal motion path is therefore:

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
HX1
    |
    v
Servo 2040 authoritative safety
```

The configured Pi command-rate values are tuning parameters.

They must not be interpreted as proof that the installed servos or mechanical system are safe at any particular physical actuator speed.