# Pi Robot Runtime

**Status:** Milestone 2 stationary energized baseline

## Purpose

`RobotRuntime` is the first Pi-side layer that owns wall-clock scheduling and
the Servo 2040 lifecycle protocol.

It composes the deterministic Milestone 1 `MotionPipeline` with `HX1Link`.

The Servo 2040 remains the authoritative hardware-safety controller.

## Milestone 2 scope

Milestone 2 deliberately does not walk.

The runtime performs this lifecycle:

```text
MotionPipeline.initialize_flat_stance()
    |
    v
open HX1 link
    |
    v
HELLO / INFO
    |
    v
GET_STATUS -> require DISARMED / NONE
    |
    v
STAGE exact flat-stance vector
    |
    v
HEARTBEAT
    |
    v
ARM
    |
    v
HEARTBEAT
    |
    v
START
    |
    v
stationary TARGET stream at 50 Hz
HEARTBEAT stream at 10 Hz
    |
    v
STOP
    |
    v
DISARM
    |
    v
GET_STATUS -> require DISARMED / NONE
    |
    v
close
```

The stationary target is the exact vector returned by
`MotionPipeline.initialize_flat_stance()`.

The runtime explicitly verifies that every Milestone 2 pipeline update remains
equal to that staged vector.

## Why START is included

`ARMED` proves that the Servo 2040 can energize and hold the staged target.

`ACTIVE` additionally proves that the production Pi runtime can sustain the
independent target and heartbeat contracts:

```text
TARGET     50 Hz nominal / 20 ms period
HEARTBEAT  10 Hz nominal / 100 ms period
```

Streaming the same flat target exercises the production timing/lifecycle path
without introducing gait motion.

## Timing

`RobotRuntime` owns a monotonic `RuntimeClock`.

The production default uses `time.monotonic()` and `time.sleep()`. Tests inject
a deterministic fake clock.

The runtime uses actual elapsed monotonic time when stepping the motion pipeline.
It does not send catch-up bursts after a late target deadline. A late scheduler
therefore remains visible to the Servo 2040 watchdog instead of being hidden by
rapidly replaying stale targets.

## Response deadlines

HELLO negotiation has a longer startup deadline.

Lifecycle commands use a shorter response deadline that remains below the
Servo 2040 200 ms ACTIVE motion watchdog. This ensures the host does not wait
indefinitely for a START response while failing to begin the target stream.

## Failure behavior

Once ARM has been transmitted, the runtime treats physical energization as
possible even if the ARM ACK has not yet been observed.

If an exception or `KeyboardInterrupt` occurs after that point:

```text
best-effort ESTOP
    ->
close link
```

If the host cannot transmit the ESTOP because the link itself has failed, the
Servo 2040 link watchdog remains the final fail-safe authority.

Normal completion uses:

```text
STOP
DISARM
```

and confirms the final `DISARMED / NONE` status before returning success.

## Physical limitation

The hobby servos have no joint-position feedback.

Therefore ARM can move a servo from its unknown unpowered physical position to
the staged flat stance. A stationary logical target does not mean that the
initial ARM transition is physically motionless.

Milestone 2 energized HIL must therefore be performed with the robot
mechanically supported.

## Deferred work

This milestone intentionally does not implement:

- DS4 command publication;
- command arbitration in the runtime loop;
- non-zero gait commands;
- reconnect/backoff;
- watchdog fault/recovery qualification;
- CLEAR_FAULT or CLEAR_ESTOP policy;
- web control;
- autonomy;
- sensors;
- telemetry aggregation;
- system service management.

Those are introduced only after the stationary production lifecycle is
validated.
