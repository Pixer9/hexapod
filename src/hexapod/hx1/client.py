"""Transport-independent Pi-side HX1 session client core.

The core owns sender sequencing, peer/session negotiation, profile verification,
and construction of valid Pi -> MCU command frames.

It deliberately performs no serial I/O and does not own the robot lifecycle or
safety supervisor. A later runtime adapter will feed incoming complete frames
into ``accept_frame()`` and send the returned outbound bytes through USB CDC.
"""

from __future__ import annotations

from dataclasses import dataclass

from .config import HX1ClientConfig
from .messages import (
    ZERO_SESSION,
    HX1Inbound,
    HX1Info,
    HX1Nack,
    HX1Outbound,
    build_estop,
    build_heartbeat,
    build_hello,
    build_simple_session_command,
    build_stage,
    build_target,
    joint_degrees_to_centidegrees,
    parse_inbound,
)
from .protocol import HX1Sequence, parse_frame


class HX1ClientError(RuntimeError):
    """Base class for client-core state/peer errors."""


class HX1SessionError(HX1ClientError):
    """A command requires a negotiated session that is not available."""


class HX1PeerMismatchError(HX1ClientError):
    """The connected Servo 2040 does not match the configured peer policy."""


class HX1UnexpectedResponseError(HX1ClientError):
    """A session-establishing response does not match the active request."""


@dataclass(frozen=True, slots=True)
class HX1PeerInfo:
    session: str
    server_minor: int
    firmware_version: str
    mcu_id: str
    profile_id: str
    profile_revision: int
    profile_hash: str
    joint_count: int
    capabilities: int
    state: str


class HX1ClientCore:
    """Pure deterministic HX1 client/session state."""

    def __init__(
        self,
        config: HX1ClientConfig,
        *,
        initial_seq: int = 0,
    ):
        self.config = config
        self._sequence = HX1Sequence(initial_seq)
        self._session: str | None = None
        self._peer: HX1PeerInfo | None = None
        self._pending_hello_seq: int | None = None

    @property
    def next_seq(self) -> int:
        return self._sequence.next_value

    @property
    def session(self) -> str | None:
        return self._session

    @property
    def peer(self) -> HX1PeerInfo | None:
        return self._peer

    @property
    def negotiated(self) -> bool:
        return self._session is not None and self._peer is not None

    @property
    def pending_hello_seq(self) -> int | None:
        return self._pending_hello_seq

    def reset_link_state(self) -> None:
        """Forget negotiated authority without rewinding sender sequence."""
        self._session = None
        self._peer = None
        self._pending_hello_seq = None

    def hello(self) -> HX1Outbound:
        """Begin a fresh HX1 session negotiation.

        Local session authority is invalidated as soon as HELLO is built. The
        MCU also invalidates old authority when it establishes the new session.
        """
        self._session = None
        self._peer = None

        seq = self._sequence.take()
        self._pending_hello_seq = seq
        return build_hello(
            seq,
            self.config.client_minor,
            self.config.expected_profile_id,
        )

    def heartbeat(self, host_uptime_ms: int) -> HX1Outbound:
        return build_heartbeat(
            self._sequence.take(),
            self._require_session(),
            host_uptime_ms,
        )

    def stage_degrees(
        self,
        joint_degrees: object,
    ) -> HX1Outbound:
        return build_stage(
            self._sequence.take(),
            self._require_session(),
            joint_degrees_to_centidegrees(joint_degrees),
        )

    def arm(self) -> HX1Outbound:
        return self._simple("ARM")

    def start(self) -> HX1Outbound:
        return self._simple("START")

    def target_degrees(
        self,
        joint_degrees: object,
    ) -> HX1Outbound:
        return build_target(
            self._sequence.take(),
            self._require_session(),
            self.config.target_period_ms,
            joint_degrees_to_centidegrees(joint_degrees),
        )

    def stop(self) -> HX1Outbound:
        return self._simple("STOP")

    def disarm(self) -> HX1Outbound:
        return self._simple("DISARM")

    def clear_estop(self) -> HX1Outbound:
        return self._simple("CLEAR_ESTOP")

    def clear_fault(self) -> HX1Outbound:
        return self._simple("CLEAR_FAULT")

    def get_status(self) -> HX1Outbound:
        return self._simple("GET_STATUS")

    def estop(self, reason: str) -> HX1Outbound:
        """Build ESTOP even before/session-loss negotiation.

        HX1 explicitly permits a zero session for ESTOP.
        """
        session = self._session if self._session is not None else ZERO_SESSION
        return build_estop(
            self._sequence.take(),
            session,
            reason,
        )

    def accept_frame(
        self,
        raw_frame: bytes | bytearray | str,
    ) -> HX1Inbound:
        """Parse one complete MCU frame and apply session negotiation effects."""
        message = parse_inbound(parse_frame(raw_frame))

        if isinstance(message, HX1Info):
            self._accept_info(message)
        elif (
            isinstance(message, HX1Nack)
            and self._pending_hello_seq is not None
            and message.ref_seq == self._pending_hello_seq
            and message.command == "HELLO"
        ):
            self._pending_hello_seq = None

        return message

    def _simple(self, command: str) -> HX1Outbound:
        return build_simple_session_command(
            self._sequence.take(),
            command,
            self._require_session(),
        )

    def _require_session(self) -> str:
        if self._session is None:
            raise HX1SessionError("HX1 command requires a negotiated session")
        return self._session

    def _accept_info(self, info: HX1Info) -> None:
        pending = self._pending_hello_seq
        if pending is None:
            raise HX1UnexpectedResponseError("received INFO without a pending HELLO")
        if info.ref_seq != pending:
            raise HX1UnexpectedResponseError(
                "INFO REF_SEQ does not match pending HELLO"
            )

        mismatches: list[str] = []

        if info.server_minor < self.config.required_server_minor:
            mismatches.append(
                "server minor "
                f"{info.server_minor} < required "
                f"{self.config.required_server_minor}"
            )

        if info.profile_id != self.config.expected_profile_id:
            mismatches.append(
                f"profile id {info.profile_id!r} != {self.config.expected_profile_id!r}"
            )

        if info.profile_revision != self.config.expected_profile_revision:
            mismatches.append(
                "profile revision "
                f"{info.profile_revision} != "
                f"{self.config.expected_profile_revision}"
            )

        if info.profile_hash != self.config.expected_profile_sha256:
            mismatches.append("profile SHA-256 mismatch")

        if info.joint_count != self.config.joint_count:
            mismatches.append(
                f"joint count {info.joint_count} != {self.config.joint_count}"
            )

        if mismatches:
            self._pending_hello_seq = None
            self._session = None
            self._peer = None
            raise HX1PeerMismatchError("; ".join(mismatches))

        self._peer = HX1PeerInfo(
            session=info.session,
            server_minor=info.server_minor,
            firmware_version=info.firmware_version,
            mcu_id=info.mcu_id,
            profile_id=info.profile_id,
            profile_revision=info.profile_revision,
            profile_hash=info.profile_hash,
            joint_count=info.joint_count,
            capabilities=info.capabilities,
            state=info.state,
        )
        self._session = info.session
        self._pending_hello_seq = None
