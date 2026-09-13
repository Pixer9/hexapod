# Pi Command-Rate Limiter

**Status:** Initial deterministic normal-command shaping baseline

## Purpose

The command-rate limiter sits after normal source arbitration and before the
locomotion controller:

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
```

It shapes how quickly normal requested body velocity can change.

It does not arbitrate sources, manage ESTOP/FAULT state, advance gait phase,
solve IK, communicate over HX1, or enforce actuator hard limits.

## Configuration

The standard baseline lives in:

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

These values are migrated from the useful v4 command-conditioning baseline.
They are tuning starting points, not qualified actuator safety limits.

The existing `config/control/motion.json` remains the normal command envelope.
The rate limiter validates every target and every produced output against that
envelope.

## Explicit deterministic time

The public boundary is:

```python
shaped = limiter.step(target, dt_s)
```

The limiter owns state but no wall clock. It does not sleep, clamp elapsed time,
or call `time.monotonic()`.

Negative or non-finite delta times are rejected. `dt_s = 0` is valid and leaves
the output unchanged.

## Translation is one vector

`vx` and `vy` are not slew-limited independently.

Instead, translation is treated as the BODY-frame velocity vector:

```text
v = (vx, vy)
```

The maximum change for one step is measured in Euclidean velocity space. From
rest, this preserves the requested direction exactly. For example, a request
toward `(80, 60)` begins in the same 4:3 direction rather than allowing one axis
to reach its target before the other.

This avoids arbitrary X/Y axis bias and preserves diagonal operator/autonomy
intent.

## Acceleration and deceleration

For non-reversing translation:

- increasing/equal requested speed uses the translation acceleration rate;
- decreasing requested speed uses the translation deceleration rate.

A zero target decelerates the current vector toward zero while retaining its
direction.

Yaw follows the same scalar acceleration/deceleration distinction.

## Direction reversal

A translation target more than 90 degrees away from the current velocity
(`current dot target < 0`) is treated as a true reversal.

The limiter first removes the current velocity using the deceleration rate. It
does not rotate the velocity vector through zero at the acceleration rate. If
the supplied `dt_s` contains time remaining after reaching zero, that remaining
time is used to accelerate toward the new target.

Scalar yaw sign reversals use the same rule.

This gives predictable behavior for requests such as:

```text
vx +80 -> vx -80
yaw +90 -> yaw -90
```

and makes reversal behavior independent of whether the same elapsed time is
provided as one step or several smaller steps.

## Reset boundary

`reset()` is an explicit hard state change:

```python
limiter.reset()               # zero
limiter.reset(command)        # validated supplied state
```

Normal command flow should use `step()`.

The future safety/runtime layer may use a hard reset when normal shaping must be
bypassed, but this limiter itself does not decide when ESTOP, FAULT, or other
safety actions occur.

## Relationship to gait blending

Command-rate limiting and gait blending are separate stages.

The rate limiter controls changes in requested BODY velocity. The locomotion
controller owns gait phase and normal flat-stance start/stop blending.

There should be exactly one normal command-rate limiter in the final Pi motion
pipeline. Additional hidden low-pass or per-axis slew stages should not be
added downstream.

## Relationship to Servo 2040 safety

This Pi limiter improves normal command continuity but is not an actuator
safety boundary.

Servo 2040 remains responsible for authoritative actuator mapping, hard limits,
watchdogs, and any qualified MCU-side actuator slew protection. A future
Pi-side joint trajectory/global time-scaling stage will separately ensure the
18-joint IK trajectory respects the chosen normal joint-rate design budget
before HX1 transmission.
