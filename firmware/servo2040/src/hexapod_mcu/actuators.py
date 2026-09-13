"""Pure actuator validation and mapping for Servo 2040 firmware.

This module contains no ServoCluster or PWM I/O.

It accepts complete canonical logical-joint vectors in integer centidegrees,
validates them atomically against the actuator profile, and maps them into
physical Servo 2040 channel order.

Hard rate checks use integer arithmetic only.
"""


class ActuatorError(ValueError):
    """Base class for actuator-target validation failures."""


class TargetShapeError(ActuatorError):
    """Target vector is incomplete or contains invalid value types."""


class PositionLimitError(ActuatorError):
    """A logical target would exceed its physical actuator envelope."""

    def __init__(self, index, name, requested_cd, minimum_cd, maximum_cd):
        self.index = index
        self.name = name
        self.requested_cd = requested_cd
        self.minimum_cd = minimum_cd
        self.maximum_cd = maximum_cd

        super().__init__(
            "%s[%d]: logical target %d cd outside %d..%d cd"
            % (name, index, requested_cd, minimum_cd, maximum_cd)
        )


class RateLimitError(ActuatorError):
    """A logical transition would exceed a hard command-rate limit."""

    def __init__(
        self,
        index,
        name,
        previous_cd,
        requested_cd,
        dt_ms,
        required_rate_cd_s,
        maximum_rate_cd_s,
    ):
        self.index = index
        self.name = name
        self.previous_cd = previous_cd
        self.requested_cd = requested_cd
        self.dt_ms = dt_ms
        self.required_rate_cd_s = required_rate_cd_s
        self.maximum_rate_cd_s = maximum_rate_cd_s

        super().__init__(
            "%s[%d]: required rate %d cd/s exceeds %d cd/s"
            % (
                name,
                index,
                required_rate_cd_s,
                maximum_rate_cd_s,
            )
        )


class UnqualifiedRateError(ActuatorError):
    """Rate validation was requested before the profile was qualified."""


def _validate_vector(profile, target):
    try:
        length = len(target)
    except TypeError:
        raise TargetShapeError("target must be a complete joint vector")

    if length != profile.joint_count:
        raise TargetShapeError(
            "target length %d does not match joint_count %d"
            % (length, profile.joint_count)
        )

    values = []

    for index, value in enumerate(target):
        if isinstance(value, bool) or not isinstance(value, int):
            raise TargetShapeError(
                "target[%d] must be an integer centidegree value" % index
            )
        values.append(value)

    return tuple(values)


def _validate_dt_ms(dt_ms):
    if isinstance(dt_ms, bool) or not isinstance(dt_ms, int):
        raise ValueError("dt_ms must be a positive integer")
    if dt_ms <= 0:
        raise ValueError("dt_ms must be > 0")


def validate_position_target(profile, target):
    """Validate a complete logical target against every hard position limit.

    Validation is pure and atomic: this function mutates no runtime state.
    """
    values = _validate_vector(profile, target)

    for joint, requested_cd in zip(profile.joints, values):
        if requested_cd < joint.logical_min_cd or requested_cd > joint.logical_max_cd:
            raise PositionLimitError(
                index=joint.index,
                name=joint.name,
                requested_cd=requested_cd,
                minimum_cd=joint.logical_min_cd,
                maximum_cd=joint.logical_max_cd,
            )

    return values


def logical_to_servo_vector(profile, target):
    """Map logical joints to servo-space values in canonical logical order."""
    logical = validate_position_target(profile, target)
    servo = []

    for joint, logical_cd in zip(profile.joints, logical):
        servo_cd = joint.offset_cd + joint.direction * logical_cd

        # Defensive second check at the physical boundary.
        if servo_cd < joint.servo_min_cd or servo_cd > joint.servo_max_cd:
            raise PositionLimitError(
                index=joint.index,
                name=joint.name,
                requested_cd=logical_cd,
                minimum_cd=joint.logical_min_cd,
                maximum_cd=joint.logical_max_cd,
            )

        servo.append(servo_cd)

    return tuple(servo)


def logical_to_channel_vector(profile, target):
    """Map a canonical logical vector to physical channel order 0..17."""
    servo_by_logical_index = logical_to_servo_vector(profile, target)
    channels = [None] * profile.joint_count

    for joint, servo_cd in zip(profile.joints, servo_by_logical_index):
        channels[joint.channel] = servo_cd

    if any(value is None for value in channels):
        raise ActuatorError("actuator profile does not cover every channel")

    return tuple(channels)


def required_rate_cd_s(previous_cd, requested_cd, dt_ms):
    """Return the minimum whole cd/s required for a transition.

    The value is rounded upward for diagnostics. Enforcement itself uses exact
    integer cross-multiplication and therefore has no floating-point ambiguity.
    """
    _validate_dt_ms(dt_ms)

    delta_cd = abs(requested_cd - previous_cd)
    numerator = delta_cd * 1000
    return (numerator + dt_ms - 1) // dt_ms


def validate_rate_transition(profile, previous_target, requested_target, dt_ms):
    """Validate a complete logical transition using actual elapsed milliseconds.

    Exact hard-limit condition for every joint:

        abs(delta_cd) * 1000 <= max_rate_cd_s * dt_ms

    This avoids floating-point math and avoids assuming every valid USB target
    arrives at exactly the nominal 20 ms period.

    The caller supplies ``dt_ms``:
      * first TARGET after START: elapsed since START;
      * later TARGETs: elapsed since the last accepted TARGET.

    Rejected transitions must not become the caller's new reference target or
    refresh its motion watchdog.
    """
    _validate_dt_ms(dt_ms)

    previous = validate_position_target(profile, previous_target)
    requested = validate_position_target(profile, requested_target)

    # Fail before checking individual deltas if the safety profile is not
    # qualified. This makes the current migrated profile explicitly unusable
    # for energized motion while still allowing position/mapping tests.
    for joint in profile.joints:
        if joint.max_rate_cd_s is None:
            raise UnqualifiedRateError(
                "%s[%d]: max_rate_cd_s is not qualified" % (joint.name, joint.index)
            )

    for joint, old_cd, new_cd in zip(profile.joints, previous, requested):
        delta_cd = abs(new_cd - old_cd)
        lhs = delta_cd * 1000
        rhs = joint.max_rate_cd_s * dt_ms

        if lhs > rhs:
            raise RateLimitError(
                index=joint.index,
                name=joint.name,
                previous_cd=old_cd,
                requested_cd=new_cd,
                dt_ms=dt_ms,
                required_rate_cd_s=required_rate_cd_s(
                    old_cd,
                    new_cd,
                    dt_ms,
                ),
                maximum_rate_cd_s=joint.max_rate_cd_s,
            )

    return requested
