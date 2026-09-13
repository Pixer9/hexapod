# Pi Joint-Trajectory Rate Scaling

**Status:** Initial deterministic normal-motion baseline

## Purpose

This layer sits after whole-body inverse kinematics and before the future HX1
transport/output layer:

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
18 canonical logical joint angles
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

Its job is to prevent the Pi's **normal commanded joint trajectory** from asking
any canonical logical joint to move faster than the configured Pi-side design
budget.

It does not replace Servo 2040 hard actuator limits, watchdogs, hard-rate
qualification, or ESTOP behavior.

## Configuration

The initial standard configuration lives in:

```text
config/control/joint-trajectory.json
```

Baseline:

```text
max_joint_rate_deg_s = 250
```

This is the existing provisional Pi design budget. It is a conservative
normal-motion target for software development and trajectory analysis.

It is **not** a qualified physical servo hard limit.

## Canonical vector

The scaler consumes the exact 18-element canonical logical joint vector produced
by `RobotKinematics`.

The order remains:

```text
RF coxa, RF femur, RF tibia,
RM coxa, RM femur, RM tibia,
RB coxa, RB femur, RB tibia,
LF coxa, LF femur, LF tibia,
LM coxa, LM femur, LM tibia,
LB coxa, LB femur, LB tibia
```

No Servo 2040 channel mapping, direction inversion, trim, or pulse conversion is
performed here.

## Global scaling rule

Let:

```text
q      = currently commanded 18-joint vector
q*     = newly requested IK vector
dq     = q* - q
R      = configured max joint rate
dt     = elapsed control time
```

The largest requested joint change is:

```text
D = max_i |dq_i|
```

The largest change allowed during this update is:

```text
A = R * dt
```

The single global scale factor is:

```text
alpha = min(1, A / D)
```

for `D > 0`.

Every joint receives the same factor:

```text
q_next = q + alpha * dq
```

Therefore, if one joint is rate-limiting, all 18 joints remain on the same
requested joint-space segment rather than some joints being independently
clipped.

This is the key distinction between global trajectory scaling and per-joint
clamping.

## Time interpretation

For the current target segment, the minimum duration implied by the design
budget is:

```text
required_duration = D / R
```

When `required_duration > dt`, the segment cannot be completed during this
update, so only `alpha < 1` of the joint-space segment is executed.

This effectively stretches execution time for that segment.

The baseline is intentionally a causal per-update scaler, not a predictive
time-optimal trajectory planner. If telemetry shows sustained scaling during
normal walking, the correct response is to revisit gait frequency, foot
trajectory tuning, command limits, or the qualified rate budget rather than
accept chronic trajectory lag.

## Explicit initialization

The scaler starts uninitialized and must be seeded with the currently commanded
canonical joint vector:

```python
scaler.seed(current_commanded_vector)
```

There is deliberately no default all-zero joint pose. Mathematical joint zero
is not a universal safe physical stance.

The seed represents the **commanded** state, not measured servo feedback.

Future runtime startup/staging logic must decide which known commanded vector is
valid before normal trajectory streaming begins.

## Deterministic timing

The public update boundary is:

```python
step = scaler.step(target_vector, dt_s)
```

No wall clock is read inside the scaler.

`dt_s = 0` is valid and applies no movement. Negative or non-finite delta times
are rejected.

For a fixed target segment, splitting an elapsed interval into smaller updates
produces the same point along that straight joint-space segment.

## Diagnostics

Each update reports:

- `scale`
- whether the update was limited
- canonical index of the joint with the largest requested delta
- largest requested joint delta
- minimum duration required by the rate budget
- peak applied joint rate

These values are intended for future runtime telemetry and the web dashboard.

## Safety boundary

This is a Pi-side **normal-motion** continuity mechanism only.

It must not be interpreted as proof that the installed DS3235 servos can safely
run at 250 degrees/second under load.

The Servo 2040 actuator profile remains authoritative for physical mapping and
hard safety. MCU hard rate values remain a separate qualification task.

ESTOP and safety faults also remain outside this normal scaler. The future
SafetySupervisor/Servo 2040 state machine determines how those events stop and
disarm physical motion.

## Why no independent joint clipping

Independent rate limiting such as:

```text
joint 0 -> clamp
joint 1 -> no clamp
joint 2 -> clamp differently
...
```

would change the relative 18-joint motion requested by IK.

The global factor keeps the instantaneous correction vector coherent. It does
not guarantee an exact Cartesian foot path when following a continuously moving
target, but it avoids the larger distortion caused by independent joint-rate
clamps.

## Relationship to future work

After this layer, the remaining path to the Servo 2040 is primarily integration
work:

```text
joint trajectory output
    -> canonical degree/centidegree serialization
    -> HX1 session/client
    -> staged/armed runtime lifecycle
    -> Servo 2040
```

Before physical walking, the MCU actuator profile still requires qualified hard
joint-rate values and the planned physical watchdog/ESTOP tests.
