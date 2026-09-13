"""Thin Servo 2040 hardware boundary.

This module is the only normal-runtime layer that knows about Pimoroni's
ServoCluster API. It intentionally contains no protocol parsing, gait, IK,
profile transformation, or runtime-state policy.

Complete channel vectors are staged with ``load=False`` and committed with one
``load()`` so all 18 PWM values change together.
"""

from .constants import JOINT_COUNT


class HardwareError(RuntimeError):
    """Base class for Servo 2040 hardware-boundary failures."""


class HardwareCompatibilityError(HardwareError):
    """The actuator profile cannot be represented by the hardware backend."""


class HardwareStateError(HardwareError):
    """A hardware operation was requested in an invalid output state."""


class ServoOutputHardware:
    """Atomic ServoCluster output adapter.

    Input values are physical channel-space integer centidegrees that have
    already been validated and mapped by ``actuators.py``.
    """

    def __init__(self, cluster, joint_count=JOINT_COUNT):
        self._cluster = cluster
        self._joint_count = int(joint_count)

        # ServoCluster construction alone does not prove the electrical state.
        # Only a successful force_disabled() establishes known-disabled PWM.
        self._enabled = None
        self._last_channel_target_cd = None

        count = int(self._cluster.count())
        if count != self._joint_count:
            raise HardwareCompatibilityError(
                "ServoCluster count %d does not match expected %d"
                % (count, self._joint_count)
            )

    @property
    def enabled(self):
        """Return True, False, or None when physical output state is unknown."""
        return self._enabled

    @property
    def last_channel_target_cd(self):
        return self._last_channel_target_cd

    def backend_value_range_cd(self, channel):
        self._validate_channel(channel)
        minimum_cd = int(round(float(self._cluster.min_value(channel)) * 100.0))
        maximum_cd = int(round(float(self._cluster.max_value(channel)) * 100.0))
        return minimum_cd, maximum_cd

    def profile_compatibility_errors(self, profile):
        errors = []

        if profile.joint_count != self._joint_count:
            errors.append(
                "profile joint_count %d does not match hardware count %d"
                % (profile.joint_count, self._joint_count)
            )
            return tuple(errors)

        for joint in profile.joints:
            backend_min_cd, backend_max_cd = self.backend_value_range_cd(joint.channel)

            if joint.servo_min_cd < backend_min_cd:
                errors.append(
                    "%s ch%d servo_min_cd %d is below backend minimum %d"
                    % (
                        joint.name,
                        joint.channel,
                        joint.servo_min_cd,
                        backend_min_cd,
                    )
                )

            if joint.servo_max_cd > backend_max_cd:
                errors.append(
                    "%s ch%d servo_max_cd %d is above backend maximum %d"
                    % (
                        joint.name,
                        joint.channel,
                        joint.servo_max_cd,
                        backend_max_cd,
                    )
                )

        return tuple(errors)

    def require_profile_compatible(self, profile):
        errors = self.profile_compatibility_errors(profile)
        if errors:
            raise HardwareCompatibilityError("; ".join(errors))
        return True

    def force_disabled(self):
        """Commit zero PWM on every channel and clear output authority.

        ``enabled`` becomes ``None`` when the backend reports a disable failure.
        Unknown is deliberately distinct from disabled: the safety runtime must
        keep retrying a disable operation until it is known to have succeeded.
        """
        try:
            self._cluster.disable_all(load=True)
        except Exception as exc:
            self._enabled = None
            raise HardwareError("failed to disable ServoCluster: %s" % exc)

        self._enabled = False
        self._last_channel_target_cd = None

    def enable_at_target(self, channel_target_cd):
        """Atomically enable all outputs at one complete prevalidated pose.

        This deliberately does not call ``enable_all()``. ServoCluster's
        default first-enable behavior is to use the calibration midpoint when
        no previous pulse exists. Instead we stage every desired target with
        ``load=False`` and commit once.
        """
        if self._enabled is not False:
            raise HardwareStateError("outputs must be known-disabled before enabling")

        target = self._normalize_channel_target(channel_target_cd)
        self._commit_target(target)
        self._enabled = True
        self._last_channel_target_cd = target

    def apply_target(self, channel_target_cd):
        """Atomically update all channels while outputs are enabled."""
        if self._enabled is not True:
            raise HardwareStateError(
                "cannot apply target unless outputs are known-enabled"
            )

        target = self._normalize_channel_target(channel_target_cd)
        self._commit_target(target)
        self._last_channel_target_cd = target

    def _commit_target(self, target):
        try:
            for channel, value_cd in enumerate(target):
                self._cluster.value(
                    channel,
                    value_cd / 100.0,
                    load=False,
                )

            self._cluster.load()
        except Exception as exc:
            try:
                self._cluster.disable_all(load=True)
            except Exception:
                # The active electrical state is no longer knowable. Preserve
                # the previous target for diagnostics and force the runtime to
                # keep attempting a disable.
                self._enabled = None
            else:
                self._enabled = False
                self._last_channel_target_cd = None

            raise HardwareError("ServoCluster target commit failed: %s" % exc)

    def _normalize_channel_target(self, target):
        try:
            length = len(target)
        except TypeError:
            raise HardwareError("channel target must be a complete vector")

        if length != self._joint_count:
            raise HardwareError(
                "channel target length %d does not match hardware count %d"
                % (length, self._joint_count)
            )

        out = []

        for channel, value_cd in enumerate(target):
            if isinstance(value_cd, bool) or not isinstance(value_cd, int):
                raise HardwareError(
                    "channel %d target must be integer centidegrees" % channel
                )

            minimum_cd, maximum_cd = self.backend_value_range_cd(channel)

            if value_cd < minimum_cd or value_cd > maximum_cd:
                raise HardwareCompatibilityError(
                    "channel %d target %d cd outside backend range %d..%d"
                    % (channel, value_cd, minimum_cd, maximum_cd)
                )

            out.append(value_cd)

        return tuple(out)

    def _validate_channel(self, channel):
        if isinstance(channel, bool) or not isinstance(channel, int):
            raise IndexError("channel must be an integer")
        if channel < 0 or channel >= self._joint_count:
            raise IndexError("channel out of range")


class DisableOnlyHardware:
    """Retain ServoCluster disable authority when full adapter init fails."""

    def __init__(self, cluster, reason):
        self._cluster = cluster
        self.reason = str(reason)
        self._enabled = None

    @property
    def enabled(self):
        return self._enabled

    @property
    def last_channel_target_cd(self):
        return None

    def force_disabled(self):
        try:
            self._cluster.disable_all(load=True)
        except Exception as exc:
            self._enabled = None
            raise HardwareError("failed to disable retained ServoCluster: %s" % exc)

        self._enabled = False

    def require_profile_compatible(self, profile):
        raise HardwareCompatibilityError(self.reason)

    def enable_at_target(self, target):
        raise HardwareStateError(self.reason)

    def apply_target(self, target):
        raise HardwareStateError(self.reason)


def create_servo2040_hardware():
    """Create the real Pimoroni Servo 2040 output adapter.

    The hardware import is intentionally local so host-side tests can import
    this module without Pimoroni's MicroPython modules being present.
    """
    try:
        from servo import ServoCluster, servo2040
    except Exception as exc:
        raise HardwareError("Pimoroni ServoCluster runtime is unavailable: %s" % exc)

    pins = list(range(servo2040.SERVO_1, servo2040.SERVO_18 + 1))

    try:
        cluster = ServoCluster(
            pio=0,
            sm=0,
            pins=pins,
            freq=50,
            auto_phase=True,
        )
    except Exception as exc:
        raise HardwareError("failed to create ServoCluster: %s" % exc)

    try:
        hardware = ServoOutputHardware(cluster)
    except Exception as exc:
        # A physical cluster exists. Keep a disable-capable handle instead of
        # discarding it just because the full output adapter is incompatible.
        hardware = DisableOnlyHardware(
            cluster,
            "ServoOutputHardware initialization failed: %s" % exc,
        )
    except BaseException:
        # Preserve process-level exceptions, but do not abandon the cluster
        # without one best-effort raw disable first.
        try:
            cluster.disable_all(load=True)
        except BaseException:
            pass
        raise

    try:
        hardware.force_disabled()
    except HardwareError:
        # Keep the real/disable-only adapter in UNKNOWN state. Runtime self-test
        # immediately retries and then keeps retrying while faulted if needed.
        pass
    except BaseException:
        try:
            cluster.disable_all(load=True)
        except BaseException:
            pass
        raise

    return hardware
