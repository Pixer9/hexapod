"""Cross-boundary HX1 contract tests for Raspberry Pi <-> Servo 2040.

These tests intentionally import both production implementations. They do not
reimplement HX1 in test code.

The goal is to fail whenever either side changes a shared wire/runtime contract
without the other side changing with it.
"""

from __future__ import annotations

from dataclasses import replace
import json
import sys
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
PI_SRC = REPO_ROOT / "src"
FIRMWARE_SRC = REPO_ROOT / "firmware" / "servo2040" / "src"

HX1_CONFIG_PATH = REPO_ROOT / "config" / "hardware" / "servo2040.json"
SOFT_LIMIT_PATH = (
    REPO_ROOT
    / "config"
    / "robots"
    / "standard-joint-soft-limits.json"
)
ACTUATOR_PROFILE_PATH = (
    FIRMWARE_SRC / "config" / "actuator-profile.json"
)

# The packages have distinct top-level names, so both production trees can
# coexist in one CPython process.
sys.path.insert(0, str(FIRMWARE_SRC))
sys.path.insert(0, str(PI_SRC))

from hexapod.hx1.client import (  # noqa: E402
    HX1ClientCore,
    HX1PeerMismatchError,
)
from hexapod.hx1.config import load_hx1_client_config  # noqa: E402
from hexapod.hx1.messages import (  # noqa: E402
    JOINT_COUNT as PI_JOINT_COUNT,
    VALID_ERRORS as PI_VALID_ERRORS,
    VALID_STATES as PI_VALID_STATES,
    HX1Ack,
    HX1Event,
    HX1Info,
    HX1Nack,
    HX1Status,
    joint_degrees_to_centidegrees,
)
from hexapod.hx1.protocol import (  # noqa: E402
    MAX_FRAME_BYTES as PI_MAX_FRAME_BYTES,
    MAX_MESSAGE_TYPE_CHARS as PI_MAX_MESSAGE_TYPE_CHARS,
    PROTOCOL_MAJOR as PI_PROTOCOL_MAJOR,
    PROTOCOL_MINOR as PI_PROTOCOL_MINOR,
    PROTOCOL_PREFIX as PI_PROTOCOL_PREFIX,
    UINT32_HALF_RANGE as PI_UINT32_HALF_RANGE,
    UINT32_MAX as PI_UINT32_MAX,
    HX1LineFramer,
    crc16_ccitt_false as pi_crc16,
    encode_frame as pi_encode_frame,
    parse_frame as pi_parse_frame,
    sequence_is_newer as pi_sequence_is_newer,
)
from hexapod.trajectory.joint_limits import (  # noqa: E402
    CANONICAL_JOINT_NAMES as PI_CANONICAL_JOINT_NAMES,
    load_joint_soft_limit_profile,
)

from hexapod_mcu.constants import (  # noqa: E402
    HEARTBEAT_RATE_HZ,
    JOINT_COUNT as MCU_JOINT_COUNT,
    LINK_TIMEOUT_MS,
    MAX_FRAME_BYTES as MCU_MAX_FRAME_BYTES,
    MAX_MESSAGE_TYPE_CHARS as MCU_MAX_MESSAGE_TYPE_CHARS,
    MOTION_TIMEOUT_MS,
    PROTOCOL_MAJOR as MCU_PROTOCOL_MAJOR,
    PROTOCOL_MINOR as MCU_PROTOCOL_MINOR,
    PROTOCOL_PREFIX as MCU_PROTOCOL_PREFIX,
    TARGET_PERIOD_MS as MCU_TARGET_PERIOD_MS,
    TARGET_RATE_HZ,
    UINT32_HALF_RANGE as MCU_UINT32_HALF_RANGE,
    UINT32_MAX as MCU_UINT32_MAX,
)
from hexapod_mcu.hardware import HardwareError  # noqa: E402
from hexapod_mcu.profile import (  # noqa: E402
    CANONICAL_JOINT_NAMES as MCU_CANONICAL_JOINT_NAMES,
    load_profile,
    parse_profile_bytes,
)
from hexapod_mcu.protocol import (  # noqa: E402
    LineFramer as MCULineFramer,
    crc16_ccitt_false as mcu_crc16,
    encode_frame as mcu_encode_frame,
    parse_frame as mcu_parse_frame,
    sequence_is_newer as mcu_sequence_is_newer,
)
from hexapod_mcu.runtime import RuntimeCoordinator  # noqa: E402
from hexapod_mcu.state_machine import (  # noqa: E402
    Error as MCUError,
    Fault as MCUFault,
    RuntimeStateMachine,
    State as MCUState,
)
from hexapod_mcu.telemetry import TelemetryEncoder  # noqa: E402


SESSION_INT = 0xA1B2C3D4
SESSION = "A1B2C3D4"

SAFE_DEGREES = tuple(
    90.0 if name.endswith("_tibia") else 0.0
    for name in PI_CANONICAL_JOINT_NAMES
)


class ContractHardware:
    """Small deterministic hardware adapter used only by host contract tests."""

    def __init__(self):
        self.enabled = False
        self.last_channel_target_cd = None

    def force_disabled(self):
        self.enabled = False
        self.last_channel_target_cd = None

    def require_profile_compatible(self, profile):
        return True

    def enable_at_target(self, target):
        self.enabled = True
        self.last_channel_target_cd = tuple(target)

    def apply_target(self, target):
        if not self.enabled:
            raise HardwareError("contract hardware is not enabled")
        self.last_channel_target_cd = tuple(target)


def _class_tokens(cls):
    return frozenset(
        value
        for name, value in vars(cls).items()
        if name.isupper() and isinstance(value, str)
    )


def _profile_variant(*, max_rate_cd_s, revision_delta):
    """Create a host-only profile variant from the real deployed profile data."""
    data = json.loads(
        ACTUATOR_PROFILE_PATH.read_text(encoding="utf-8")
    )
    data["profile_revision"] += revision_delta

    for joint in data["joints"]:
        joint["max_rate_cd_s"] = max_rate_cd_s

    raw = json.dumps(
        data,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")

    return parse_profile_bytes(raw)


def _config_for_profile(base_config, profile):
    return replace(
        base_config,
        expected_profile_id=profile.profile_id,
        expected_profile_revision=profile.profile_revision,
        expected_profile_sha256=profile.profile_hash,
        joint_count=profile.joint_count,
    )


def _make_runtime(profile):
    state_machine = RuntimeStateMachine()
    hardware = ContractHardware()
    telemetry = TelemetryEncoder(
        profile=profile,
        firmware_version="contract-test",
        mcu_id="e661410403724132",
        capabilities=0,
    )
    runtime = RuntimeCoordinator(
        profile=profile,
        state_machine=state_machine,
        hardware=hardware,
        telemetry=telemetry,
        session_factory=lambda: SESSION_INT,
    )
    return runtime, state_machine, hardware


def _negotiate(client, runtime, *, now_ms=10):
    hello = client.hello()
    responses = runtime.handle_frame(hello.frame, now_ms)

    if len(responses) != 1:
        raise AssertionError(
            "HELLO must produce exactly one INFO in contract flow"
        )

    info = client.accept_frame(responses[0])
    if not isinstance(info, HX1Info):
        raise AssertionError("HELLO response was not typed as HX1Info")

    return hello, info


class ProtocolPrimitiveContractTests(unittest.TestCase):
    def test_protocol_identity_bounds_and_vocabularies_match(self):
        self.assertEqual(PI_PROTOCOL_PREFIX, MCU_PROTOCOL_PREFIX)
        self.assertEqual(PI_PROTOCOL_MAJOR, MCU_PROTOCOL_MAJOR)
        self.assertEqual(PI_PROTOCOL_MINOR, MCU_PROTOCOL_MINOR)

        self.assertEqual(PI_MAX_FRAME_BYTES, MCU_MAX_FRAME_BYTES)
        self.assertEqual(
            PI_MAX_MESSAGE_TYPE_CHARS,
            MCU_MAX_MESSAGE_TYPE_CHARS,
        )
        self.assertEqual(PI_UINT32_MAX, MCU_UINT32_MAX)
        self.assertEqual(
            PI_UINT32_HALF_RANGE,
            MCU_UINT32_HALF_RANGE,
        )
        self.assertEqual(PI_JOINT_COUNT, MCU_JOINT_COUNT)

        # Pi rejects unknown MCU states, so state vocabulary drift must be
        # deliberate and cross-reviewed.
        self.assertEqual(
            PI_VALID_STATES,
            _class_tokens(MCUState),
        )

        # Pi can preserve unknown NACK identifiers, but keep the currently
        # documented vocabulary synchronized as a release gate.
        self.assertEqual(
            PI_VALID_ERRORS,
            _class_tokens(MCUError),
        )

    def test_crc_and_encoded_bytes_match(self):
        self.assertEqual(pi_crc16(b"123456789"), 0x29B1)
        self.assertEqual(mcu_crc16(b"123456789"), 0x29B1)

        safe_cd = joint_degrees_to_centidegrees(SAFE_DEGREES)
        vectors = (
            (
                0,
                "HELLO",
                (1, "hexapod-standard-v1"),
            ),
            (
                42,
                "STOP",
                (SESSION,),
            ),
            (
                PI_UINT32_MAX,
                "TARGET",
                (SESSION, 20, *safe_cd),
            ),
            (
                77,
                "ESTOP",
                ("00000000", "contract_test"),
            ),
        )

        for seq, message_type, fields in vectors:
            with self.subTest(
                seq=seq,
                message_type=message_type,
            ):
                self.assertEqual(
                    pi_encode_frame(
                        seq,
                        message_type,
                        *fields,
                    ),
                    mcu_encode_frame(
                        seq,
                        message_type,
                        *fields,
                    ),
                )

    def test_each_parser_accepts_the_other_side_encoder(self):
        safe_cd = joint_degrees_to_centidegrees(SAFE_DEGREES)
        vectors = (
            (
                0,
                "HELLO",
                (1, "hexapod-standard-v1"),
            ),
            (
                9,
                "STAGE",
                (SESSION, *safe_cd),
            ),
            (
                10,
                "TARGET",
                (SESSION, 20, *safe_cd),
            ),
            (
                11,
                "NACK",
                (10, "TARGET", "ERR_RATE_LIMIT"),
            ),
        )

        for seq, message_type, fields in vectors:
            with self.subTest(
                seq=seq,
                message_type=message_type,
            ):
                pi_parsed = pi_parse_frame(
                    mcu_encode_frame(
                        seq,
                        message_type,
                        *fields,
                    )
                )
                self.assertEqual(pi_parsed.seq, seq)
                self.assertEqual(
                    pi_parsed.message_type,
                    message_type,
                )
                self.assertEqual(
                    pi_parsed.fields,
                    tuple(str(value) for value in fields),
                )

                mcu_parsed = mcu_parse_frame(
                    pi_encode_frame(
                        seq,
                        message_type,
                        *fields,
                    )
                )
                self.assertEqual(
                    mcu_parsed,
                    (
                        seq,
                        message_type,
                        tuple(str(value) for value in fields),
                    ),
                )

    def test_sequence_half_range_ordering_matches(self):
        cases = (
            (1, 0),
            (0, PI_UINT32_MAX),
            (PI_UINT32_MAX, 0),
            (0x7FFFFFFF, 0),
            (0x80000000, 0),
            (5, 5),
        )

        for candidate, previous in cases:
            with self.subTest(
                candidate=candidate,
                previous=previous,
            ):
                self.assertEqual(
                    pi_sequence_is_newer(
                        candidate,
                        previous,
                    ),
                    mcu_sequence_is_newer(
                        candidate,
                        previous,
                    ),
                )

    def test_line_framers_match_fragmentation_and_oversize_recovery(self):
        frame_a = pi_encode_frame(
            1,
            "ACK",
            0,
            "HELLO",
        )
        frame_b = pi_encode_frame(
            2,
            "EVENT",
            "FAULT",
            "PROFILE",
        )

        pi_framer = HX1LineFramer()
        mcu_framer = MCULineFramer()

        pi_frames = []
        mcu_frames = []

        chunks = (
            frame_a[:5],
            frame_a[5:] + frame_b[:7],
            frame_b[7:],
        )

        for chunk in chunks:
            pi_frames.extend(pi_framer.feed(chunk))
            mcu_frames.extend(mcu_framer.feed(chunk))

        self.assertEqual(
            tuple(pi_frames),
            (frame_a, frame_b),
        )
        self.assertEqual(
            tuple(mcu_frames),
            (frame_a, frame_b),
        )

        pi_framer = HX1LineFramer()
        mcu_framer = MCULineFramer()

        stream = (
            b"X" * (PI_MAX_FRAME_BYTES + 1)
            + b"\n"
            + frame_a
        )

        self.assertEqual(
            tuple(pi_framer.feed(stream)),
            (frame_a,),
        )
        self.assertEqual(
            tuple(mcu_framer.feed(stream)),
            (frame_a,),
        )
        self.assertEqual(pi_framer.framing_errors, 1)
        self.assertEqual(mcu_framer.framing_errors, 1)


class ProfileAndTimingContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = load_hx1_client_config(
            HX1_CONFIG_PATH
        )
        cls.profile = load_profile(
            ACTUATOR_PROFILE_PATH
        )

    def test_pi_link_policy_matches_exact_current_mcu_profile(self):
        self.assertEqual(
            self.config.expected_profile_id,
            self.profile.profile_id,
        )
        self.assertEqual(
            self.config.expected_profile_revision,
            self.profile.profile_revision,
        )
        self.assertEqual(
            self.config.expected_profile_sha256,
            self.profile.profile_hash,
        )
        self.assertEqual(
            self.config.joint_count,
            self.profile.joint_count,
        )

    def test_canonical_joint_names_and_soft_envelope_match_mcu_profile(self):
        soft = load_joint_soft_limit_profile(
            SOFT_LIMIT_PATH
        )

        self.assertEqual(
            PI_CANONICAL_JOINT_NAMES,
            MCU_CANONICAL_JOINT_NAMES,
        )
        self.assertEqual(
            tuple(joint.name for joint in self.profile.joints),
            PI_CANONICAL_JOINT_NAMES,
        )

        for soft_joint, hard_joint in zip(
            soft.joints,
            self.profile.joints,
        ):
            with self.subTest(joint=soft_joint.name):
                self.assertEqual(
                    soft_joint.index,
                    hard_joint.index,
                )
                self.assertEqual(
                    soft_joint.name,
                    hard_joint.name,
                )

                soft_min_cd = int(
                    round(soft_joint.min_deg * 100.0)
                )
                soft_max_cd = int(
                    round(soft_joint.max_deg * 100.0)
                )

                self.assertGreaterEqual(
                    soft_min_cd,
                    hard_joint.logical_min_cd,
                )
                self.assertLessEqual(
                    soft_max_cd,
                    hard_joint.logical_max_cd,
                )

    def test_nominal_timing_matches_firmware_contract(self):
        self.assertEqual(
            self.config.heartbeat_period_ms,
            1000 // HEARTBEAT_RATE_HZ,
        )
        self.assertEqual(
            self.config.target_period_ms,
            MCU_TARGET_PERIOD_MS,
        )
        self.assertEqual(
            MCU_TARGET_PERIOD_MS,
            1000 // TARGET_RATE_HZ,
        )

        self.assertLess(
            self.config.heartbeat_period_ms,
            LINK_TIMEOUT_MS,
        )
        self.assertLess(
            self.config.target_period_ms,
            MOTION_TIMEOUT_MS,
        )
        self.assertLessEqual(
            self.config.required_server_minor,
            MCU_PROTOCOL_MINOR,
        )


class WireIntegrationContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.base_config = load_hx1_client_config(
            HX1_CONFIG_PATH
        )

    def test_all_pi_command_builders_have_mcu_compatible_schemas(self):
        profile = _profile_variant(
            max_rate_cd_s=25000,
            revision_delta=1,
        )
        config = _config_for_profile(
            self.base_config,
            profile,
        )
        runtime, _, _ = _make_runtime(profile)
        self.assertTrue(runtime.perform_self_test(0))

        client = HX1ClientCore(config)

        hello = client.hello()
        seq, message_type, fields = mcu_parse_frame(
            hello.frame
        )
        self.assertEqual(seq, hello.seq)
        self.assertEqual(message_type, "HELLO")
        self.assertEqual(len(fields), 2)

        responses = runtime.handle_frame(
            hello.frame,
            10,
        )
        self.assertEqual(len(responses), 1)
        client.accept_frame(responses[0])
        self.assertTrue(client.negotiated)

        commands = (
            (client.heartbeat(123), 2),
            (client.stage_degrees(SAFE_DEGREES), 19),
            (client.arm(), 1),
            (client.start(), 1),
            (client.target_degrees(SAFE_DEGREES), 20),
            (client.stop(), 1),
            (client.disarm(), 1),
            (client.clear_estop(), 1),
            (client.clear_fault(), 1),
            (client.get_status(), 1),
            (client.estop("contract_test"), 2),
        )

        for outbound, expected_field_count in commands:
            with self.subTest(command=outbound.command):
                seq, message_type, fields = mcu_parse_frame(
                    outbound.frame
                )
                self.assertEqual(seq, outbound.seq)
                self.assertEqual(
                    message_type,
                    outbound.command,
                )
                self.assertEqual(
                    len(fields),
                    expected_field_count,
                )

    def test_periodic_prehello_status_is_parseable_by_pi(self):
        profile = load_profile(
            ACTUATOR_PROFILE_PATH
        )
        runtime, _, _ = _make_runtime(profile)

        # The current profile may be qualified in a future revision. Either
        # DISARMED or FAULT is acceptable here; the contract under test is that
        # pre-session periodic STATUS remains parseable.
        runtime.perform_self_test(0)

        client = HX1ClientCore(self.base_config)
        status = client.accept_frame(
            runtime.status_frame(10)
        )

        self.assertIsInstance(status, HX1Status)
        self.assertEqual(status.session, "00000000")
        self.assertFalse(status.command_valid)
        self.assertIsNone(status.commanded_joint_cd)
        self.assertFalse(client.negotiated)

    def test_unqualified_profile_diagnostic_round_trip(self):
        profile = _profile_variant(
            max_rate_cd_s=None,
            revision_delta=2,
        )
        config = _config_for_profile(
            self.base_config,
            profile,
        )
        runtime, state_machine, hardware = _make_runtime(
            profile
        )

        self.assertFalse(runtime.perform_self_test(0))
        self.assertEqual(
            state_machine.state,
            MCUState.FAULT,
        )
        self.assertEqual(
            state_machine.fault,
            MCUFault.PROFILE,
        )
        self.assertFalse(hardware.enabled)

        client = HX1ClientCore(config)
        hello, info = _negotiate(
            client,
            runtime,
            now_ms=10,
        )

        self.assertEqual(info.ref_seq, hello.seq)
        self.assertEqual(info.state, MCUState.FAULT)
        self.assertTrue(client.negotiated)
        self.assertEqual(client.session, SESSION)

        request = client.get_status()
        responses = runtime.handle_frame(
            request.frame,
            20,
        )
        self.assertEqual(len(responses), 1)

        status = client.accept_frame(responses[0])

        self.assertIsInstance(status, HX1Status)
        self.assertEqual(
            status.session,
            client.session,
        )
        self.assertEqual(
            status.state,
            MCUState.FAULT,
        )
        self.assertEqual(
            status.fault,
            MCUFault.PROFILE,
        )
        self.assertFalse(status.command_valid)
        self.assertIsNone(status.commanded_joint_cd)
        self.assertFalse(hardware.enabled)

    def test_profile_mismatch_is_rejected_by_pi_after_mcu_info(self):
        profile = load_profile(
            ACTUATOR_PROFILE_PATH
        )
        runtime, state_machine, _ = _make_runtime(
            profile
        )
        runtime.perform_self_test(0)

        bad_config = replace(
            self.base_config,
            expected_profile_id="wrong-profile",
        )
        client = HX1ClientCore(bad_config)

        hello = client.hello()
        responses = runtime.handle_frame(
            hello.frame,
            10,
        )
        self.assertEqual(len(responses), 1)

        with self.assertRaises(HX1PeerMismatchError):
            client.accept_frame(responses[0])

        # The MCU has created a diagnostic session, but it marks the session as
        # profile-mismatched and the Pi retains no local control authority.
        self.assertEqual(
            state_machine.session_id,
            SESSION,
        )
        self.assertFalse(
            state_machine.session_profile_match
        )
        self.assertFalse(client.negotiated)
        self.assertIsNone(client.session)

    def test_mcu_nack_is_typed_by_pi(self):
        profile = _profile_variant(
            max_rate_cd_s=25000,
            revision_delta=3,
        )
        config = _config_for_profile(
            self.base_config,
            profile,
        )
        runtime, state_machine, hardware = _make_runtime(
            profile
        )
        self.assertTrue(runtime.perform_self_test(0))

        client = HX1ClientCore(config)
        _negotiate(
            client,
            runtime,
            now_ms=10,
        )

        # ARM before STAGE is semantically valid framing but not ready.
        outbound = client.arm()
        responses = runtime.handle_frame(
            outbound.frame,
            20,
        )
        self.assertEqual(len(responses), 1)

        nack = client.accept_frame(responses[0])

        self.assertIsInstance(nack, HX1Nack)
        self.assertEqual(nack.ref_seq, outbound.seq)
        self.assertEqual(nack.command, "ARM")
        self.assertEqual(nack.error, "ERR_NOT_READY")
        self.assertEqual(
            state_machine.state,
            MCUState.DISARMED,
        )
        self.assertFalse(hardware.enabled)

    def test_zero_session_estop_round_trip(self):
        profile = load_profile(
            ACTUATOR_PROFILE_PATH
        )
        runtime, state_machine, hardware = _make_runtime(
            profile
        )
        runtime.perform_self_test(0)

        client = HX1ClientCore(self.base_config)
        outbound = client.estop("contract_test")

        _, message_type, fields = mcu_parse_frame(
            outbound.frame
        )
        self.assertEqual(message_type, "ESTOP")
        self.assertEqual(fields[0], "00000000")

        responses = runtime.handle_frame(
            outbound.frame,
            10,
        )
        self.assertEqual(len(responses), 2)

        ack = client.accept_frame(responses[0])
        event = client.accept_frame(responses[1])

        self.assertIsInstance(ack, HX1Ack)
        self.assertEqual(ack.ref_seq, outbound.seq)
        self.assertEqual(ack.command, "ESTOP")

        self.assertIsInstance(event, HX1Event)
        self.assertEqual(event.event_type, "ESTOP")
        self.assertEqual(event.detail, "contract_test")

        self.assertEqual(
            state_machine.state,
            MCUState.ESTOP,
        )
        self.assertFalse(hardware.enabled)

    def test_qualified_control_lifecycle_round_trip(self):
        profile = _profile_variant(
            max_rate_cd_s=25000,
            revision_delta=4,
        )
        config = _config_for_profile(
            self.base_config,
            profile,
        )
        runtime, state_machine, hardware = _make_runtime(
            profile
        )

        self.assertTrue(runtime.perform_self_test(0))
        self.assertEqual(
            state_machine.state,
            MCUState.DISARMED,
        )

        client = HX1ClientCore(config)
        hello, info = _negotiate(
            client,
            runtime,
            now_ms=10,
        )
        self.assertEqual(info.ref_seq, hello.seq)
        self.assertEqual(info.state, MCUState.DISARMED)

        stage = client.stage_degrees(SAFE_DEGREES)
        responses = runtime.handle_frame(
            stage.frame,
            20,
        )
        self.assertEqual(len(responses), 1)
        stage_ack = client.accept_frame(
            responses[0]
        )
        self.assertIsInstance(stage_ack, HX1Ack)
        self.assertEqual(stage_ack.command, "STAGE")

        heartbeat = client.heartbeat(30)
        self.assertEqual(
            runtime.handle_frame(
                heartbeat.frame,
                30,
            ),
            (),
        )

        arm = client.arm()
        responses = runtime.handle_frame(
            arm.frame,
            40,
        )
        self.assertEqual(len(responses), 1)
        arm_ack = client.accept_frame(
            responses[0]
        )
        self.assertIsInstance(arm_ack, HX1Ack)
        self.assertEqual(arm_ack.command, "ARM")
        self.assertTrue(hardware.enabled)

        start = client.start()
        responses = runtime.handle_frame(
            start.frame,
            50,
        )
        self.assertEqual(len(responses), 1)
        start_ack = client.accept_frame(
            responses[0]
        )
        self.assertIsInstance(start_ack, HX1Ack)
        self.assertEqual(start_ack.command, "START")
        self.assertEqual(
            state_machine.state,
            MCUState.ACTIVE,
        )

        target_deg = list(SAFE_DEGREES)
        target_deg[0] = 0.1

        target = client.target_degrees(
            target_deg
        )
        self.assertEqual(
            runtime.handle_frame(
                target.frame,
                70,
            ),
            (),
        )

        status_request = client.get_status()
        responses = runtime.handle_frame(
            status_request.frame,
            80,
        )
        self.assertEqual(len(responses), 1)

        status = client.accept_frame(
            responses[0]
        )
        self.assertIsInstance(status, HX1Status)
        self.assertEqual(
            status.state,
            MCUState.ACTIVE,
        )
        self.assertEqual(
            status.fault,
            MCUFault.NONE,
        )
        self.assertEqual(
            status.last_target_seq,
            target.seq,
        )
        self.assertTrue(status.command_valid)
        self.assertEqual(
            status.commanded_joint_cd,
            joint_degrees_to_centidegrees(
                target_deg
            ),
        )

        stop = client.stop()
        responses = runtime.handle_frame(
            stop.frame,
            90,
        )
        self.assertEqual(len(responses), 1)
        stop_ack = client.accept_frame(
            responses[0]
        )
        self.assertIsInstance(stop_ack, HX1Ack)
        self.assertEqual(stop_ack.command, "STOP")
        self.assertEqual(
            state_machine.state,
            MCUState.ARMED,
        )
        self.assertTrue(hardware.enabled)

        disarm = client.disarm()
        responses = runtime.handle_frame(
            disarm.frame,
            100,
        )
        self.assertEqual(len(responses), 1)
        disarm_ack = client.accept_frame(
            responses[0]
        )
        self.assertIsInstance(disarm_ack, HX1Ack)
        self.assertEqual(
            disarm_ack.command,
            "DISARM",
        )

        self.assertEqual(
            state_machine.state,
            MCUState.DISARMED,
        )
        self.assertFalse(hardware.enabled)


if __name__ == "__main__":
    unittest.main()
