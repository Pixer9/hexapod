"""Actuator-profile loading and validation for Servo 2040 firmware.

The loader deliberately separates two concepts:

1. structural validity -- the profile is well-formed and internally coherent;
2. arm qualification -- every safety parameter required for energized operation
   has been physically qualified.

The current migrated profile is expected to be structurally valid but *not*
arm-qualified because ``max_rate_cd_s`` remains null.
"""

try:
    import ujson as json
except ImportError:
    import json

try:
    import uhashlib as hashlib
except ImportError:
    import hashlib

from .constants import JOINT_COUNT


SCHEMA_VERSION = 1
ANGLE_UNIT = "centidegree"
RATE_UNIT = "centidegree_per_second"

CANONICAL_JOINT_NAMES = (
    "rf_coxa",
    "rf_femur",
    "rf_tibia",
    "rm_coxa",
    "rm_femur",
    "rm_tibia",
    "rb_coxa",
    "rb_femur",
    "rb_tibia",
    "lf_coxa",
    "lf_femur",
    "lf_tibia",
    "lm_coxa",
    "lm_femur",
    "lm_tibia",
    "lb_coxa",
    "lb_femur",
    "lb_tibia",
)

_ALLOWED_ID_CHARS = (
    "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
    "abcdefghijklmnopqrstuvwxyz"
    "0123456789"
    "._-:"
)


class ProfileError(ValueError):
    """The actuator profile is structurally invalid."""


class ProfileQualificationError(ProfileError):
    """The actuator profile is valid but not qualified for arming."""


class JointProfile:
    """Immutable-by-convention validated actuator entry."""

    def __init__(
        self,
        index,
        name,
        channel,
        direction,
        offset_cd,
        servo_min_cd,
        servo_max_cd,
        max_rate_cd_s,
    ):
        self.index = index
        self.name = name
        self.channel = channel
        self.direction = direction
        self.offset_cd = offset_cd
        self.servo_min_cd = servo_min_cd
        self.servo_max_cd = servo_max_cd
        self.max_rate_cd_s = max_rate_cd_s

        logical_a = direction * (servo_min_cd - offset_cd)
        logical_b = direction * (servo_max_cd - offset_cd)

        self.logical_min_cd = min(logical_a, logical_b)
        self.logical_max_cd = max(logical_a, logical_b)

    @property
    def arm_qualified(self):
        return self.max_rate_cd_s is not None and self.max_rate_cd_s > 0


class ActuatorProfile:
    """Validated actuator profile plus exact-byte fingerprint."""

    def __init__(
        self,
        schema_version,
        profile_id,
        profile_revision,
        joint_count,
        angle_unit,
        rate_unit,
        joints,
        profile_hash,
    ):
        self.schema_version = schema_version
        self.profile_id = profile_id
        self.profile_revision = profile_revision
        self.joint_count = joint_count
        self.angle_unit = angle_unit
        self.rate_unit = rate_unit
        self.joints = tuple(joints)
        self.profile_hash = profile_hash

    @property
    def arm_qualified(self):
        return all(joint.arm_qualified for joint in self.joints)

    def qualification_errors(self):
        errors = []

        for joint in self.joints:
            if joint.max_rate_cd_s is None:
                errors.append("%s: max_rate_cd_s is unqualified" % joint.name)
            elif joint.max_rate_cd_s <= 0:
                errors.append("%s: max_rate_cd_s must be positive" % joint.name)

        return tuple(errors)

    def require_arm_qualified(self):
        errors = self.qualification_errors()
        if errors:
            raise ProfileQualificationError("; ".join(errors))
        return True

    def joint(self, index):
        if not isinstance(index, int) or isinstance(index, bool):
            raise IndexError("joint index must be an integer")
        if index < 0 or index >= self.joint_count:
            raise IndexError("joint index out of range")
        return self.joints[index]


def sha256_hex(raw_bytes):
    """Return uppercase SHA-256 hex for exact bytes."""
    if not isinstance(raw_bytes, (bytes, bytearray)):
        raise TypeError("sha256_hex expects bytes or bytearray")

    digest = hashlib.sha256(bytes(raw_bytes)).digest()
    return "".join("%02X" % byte for byte in digest)


def load_profile(path):
    """Load and structurally validate a JSON actuator profile from disk."""
    with open(path, "rb") as handle:
        raw = handle.read()

    return parse_profile_bytes(raw)


def parse_profile_bytes(raw):
    """Parse exact JSON bytes and return a structurally validated profile."""
    if not isinstance(raw, (bytes, bytearray)):
        raise ProfileError("profile input must be bytes")

    raw = bytes(raw)

    try:
        text = raw.decode("utf-8")
    except UnicodeDecodeError:
        raise ProfileError("profile must be UTF-8 JSON")

    try:
        data = json.loads(text)
    except Exception as exc:
        raise ProfileError("profile is not valid JSON: %s" % exc)

    if not isinstance(data, dict):
        raise ProfileError("profile root must be an object")

    schema_version = _require_int(data, "schema_version", minimum=1)
    if schema_version != SCHEMA_VERSION:
        raise ProfileError(
            "unsupported schema_version %s (expected %s)"
            % (schema_version, SCHEMA_VERSION)
        )

    profile_id = _require_identifier(data, "profile_id")
    profile_revision = _require_int(data, "profile_revision", minimum=1)

    joint_count = _require_int(data, "joint_count", minimum=1)
    if joint_count != JOINT_COUNT:
        raise ProfileError(
            "joint_count must be %d for this firmware" % JOINT_COUNT
        )

    angle_unit = _require_string(data, "angle_unit")
    if angle_unit != ANGLE_UNIT:
        raise ProfileError("unsupported angle_unit %r" % angle_unit)

    rate_unit = _require_string(data, "rate_unit")
    if rate_unit != RATE_UNIT:
        raise ProfileError("unsupported rate_unit %r" % rate_unit)

    raw_joints = data.get("joints")
    if not isinstance(raw_joints, list):
        raise ProfileError("joints must be an array")

    if len(raw_joints) != joint_count:
        raise ProfileError(
            "joints length %d does not match joint_count %d"
            % (len(raw_joints), joint_count)
        )

    joints = []
    seen_channels = set()
    seen_names = set()

    for expected_index, raw_joint in enumerate(raw_joints):
        if not isinstance(raw_joint, dict):
            raise ProfileError("joint %d must be an object" % expected_index)

        index = _require_int(raw_joint, "index", minimum=0)
        if index != expected_index:
            raise ProfileError(
                "joint index %d is out of canonical order; expected %d"
                % (index, expected_index)
            )

        name = _require_string(raw_joint, "name")
        expected_name = CANONICAL_JOINT_NAMES[expected_index]
        if name != expected_name:
            raise ProfileError(
                "joint %d name %r does not match canonical name %r"
                % (expected_index, name, expected_name)
            )

        if name in seen_names:
            raise ProfileError("duplicate joint name %r" % name)
        seen_names.add(name)

        channel = _require_int(raw_joint, "channel", minimum=0, maximum=17)
        if channel in seen_channels:
            raise ProfileError("duplicate Servo 2040 channel %d" % channel)
        seen_channels.add(channel)

        direction = _require_int(raw_joint, "direction")
        if direction not in (-1, 1):
            raise ProfileError(
                "%s: direction must be exactly -1 or +1" % name
            )

        offset_cd = _require_int(raw_joint, "offset_cd")
        servo_min_cd = _require_int(raw_joint, "servo_min_cd")
        servo_max_cd = _require_int(raw_joint, "servo_max_cd")

        if servo_min_cd >= servo_max_cd:
            raise ProfileError(
                "%s: servo_min_cd must be less than servo_max_cd" % name
            )

        max_rate_cd_s = raw_joint.get("max_rate_cd_s")
        if max_rate_cd_s is not None:
            if (
                isinstance(max_rate_cd_s, bool)
                or not isinstance(max_rate_cd_s, int)
            ):
                raise ProfileError(
                    "%s: max_rate_cd_s must be an integer or null" % name
                )
            if max_rate_cd_s <= 0:
                raise ProfileError(
                    "%s: max_rate_cd_s must be positive when specified" % name
                )

        joint = JointProfile(
            index=index,
            name=name,
            channel=channel,
            direction=direction,
            offset_cd=offset_cd,
            servo_min_cd=servo_min_cd,
            servo_max_cd=servo_max_cd,
            max_rate_cd_s=max_rate_cd_s,
        )

        if joint.logical_min_cd >= joint.logical_max_cd:
            raise ProfileError("%s: derived logical range is empty" % name)

        joints.append(joint)

    return ActuatorProfile(
        schema_version=schema_version,
        profile_id=profile_id,
        profile_revision=profile_revision,
        joint_count=joint_count,
        angle_unit=angle_unit,
        rate_unit=rate_unit,
        joints=joints,
        profile_hash=sha256_hex(raw),
    )


def _require_int(mapping, key, minimum=None, maximum=None):
    if key not in mapping:
        raise ProfileError("missing required field %r" % key)

    value = mapping[key]
    if isinstance(value, bool) or not isinstance(value, int):
        raise ProfileError("%s must be an integer" % key)

    if minimum is not None and value < minimum:
        raise ProfileError("%s must be >= %d" % (key, minimum))

    if maximum is not None and value > maximum:
        raise ProfileError("%s must be <= %d" % (key, maximum))

    return value


def _require_string(mapping, key):
    if key not in mapping:
        raise ProfileError("missing required field %r" % key)

    value = mapping[key]
    if not isinstance(value, str) or not value:
        raise ProfileError("%s must be a non-empty string" % key)

    try:
        value.encode("ascii")
    except UnicodeEncodeError:
        raise ProfileError("%s must be ASCII" % key)

    return value


def _require_identifier(mapping, key):
    value = _require_string(mapping, key)

    for char in value:
        if char not in _ALLOWED_ID_CHARS:
            raise ProfileError("%s contains an invalid character" % key)

    return value
