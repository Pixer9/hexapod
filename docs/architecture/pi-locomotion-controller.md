# Pi Locomotion Controller

**Status:** Initial deterministic phase/transition baseline

## Purpose

The locomotion controller sits around the pure tripod gait sampler and owns the
small amount of state required to advance gait phase and perform normal
start/stop transitions.

```text
selected / shaped MotionCommand
    |
    v
LocomotionController
    |  explicit dt_s
    |  phase + gait envelope
    v
TripodGait
    |
    v
six BODY-frame foot targets
    |
    v
RobotKinematics
```

It does not read a wall clock. The caller supplies `dt_s` on every step, so the
same command/delta-time sequence produces the same phase, blend, and foot
targets.

## Internal modes

The controller has four locomotion-local modes:

```text
IDLE -> STARTING -> MOVING -> STOPPING -> IDLE
```

These are **not** the robot lifecycle states (`BOOTING`, `SELF_TEST`,
`DISARMED`, `ARMED`, `ACTIVE`, `FAULT`, `ESTOP`). They describe only the local
gait transition.

When idle:

- phase is exactly `0`;
- blend is exactly `0`;
- the trajectory command is zero;
- all feet are at the configured flat stance anchors.

Resetting to phase zero after a completed stop makes every later start
deterministic.

## Phase advancement

While STARTING, MOVING, or STOPPING:

```text
phase += dt_s * cycle_hz
phase %= 1
```

Phase does not advance while IDLE.

The locomotion controller owns phase state. `TripodGait` remains a pure sampler
that receives an explicit phase.

## Start/stop envelope

Transition timing lives in:

```text
config/locomotion/controller.json
```

Initial baseline:

```text
start_blend_s = 0.45
stop_blend_s  = 0.45
```

The 0.45 second start baseline carries forward the useful v4 gait-start tuning.
The stop duration is a new symmetric baseline and remains configuration to be
physically tuned later.

The internal transition progress is linear in time, but the blend sent to the
gait sampler uses quintic smootherstep:

```text
blend(u) = 6u^5 - 15u^4 + 10u^3
```

This gives zero first and second derivatives at the endpoints of a normal
start/stop envelope.

A zero duration is valid and requests an immediate normal transition.

## Normal stopping and command retention

`TripodGait.sample()` intentionally maps an actual zero `MotionCommand` directly
to flat stance. That is correct for the pure sampler, but it means a controller
cannot fade a zero command over time.

Therefore, when a **normal** motion request becomes zero, the locomotion
controller enters STOPPING and retains the last non-zero command only as the
trajectory shape being faded out:

```text
requested command  = zero
trajectory command = last non-zero command
blend              = 1 -> 0
```

Once blend reaches zero, the retained command is discarded and phase resets to
zero.

If motion is requested again during STOPPING, the same envelope progress is
reused and the controller reverses into STARTING instead of dropping to zero
and restarting the gait phase.

## Safety boundary

This stop blend is only for normal locomotion.

`ESTOP`, safety `STOP`, communication faults, or other lifecycle faults must
not wait for this controller to finish a normal gait transition. The future
SafetySupervisor/runtime layer owns those higher-priority actions.

This preserves the project rule:

```text
e-stop > DS4 > web > autonomy > idle
```

and keeps safety state separate from gait cosmetics.

## Command-rate limiting

This controller does **not** smooth or slew `vx`, `vy`, or yaw commands.

The already-planned command-rate limiter remains a separate upstream stage:

```text
arbiter
  -> command-rate limiter
  -> LocomotionController
  -> TripodGait
  -> IK
```

Until that limiter is implemented, tests use deterministic command sequences.
A sudden non-zero command change while already moving may therefore change foot
targets suddenly; that is intentionally not hidden inside the gait controller.

There must be only one normal command-rate limiting stage in the final motion
pipeline.

## Deterministic timing contract

`step(command, dt_s)` rejects negative or non-finite delta times.

The controller does not:

- call `time.monotonic()`;
- sleep;
- choose a loop frequency;
- silently clamp elapsed time;
- resynchronize a scheduler.

Those responsibilities belong to the future runtime loop. This keeps the
locomotion math directly testable without hardware or real time.
