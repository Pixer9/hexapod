"""Minimal production Pi runtime for stationary energized HX1 operation.

Milestone 2 intentionally keeps motion stationary. The runtime composes the
Milestone 1 motion pipeline with the synchronous HX1 link and owns the impure
time/lifecycle work needed to exercise the real Servo 2040 safely:

    open
    HELLO / INFO
    GET_STATUS
    STAGE configured flat stance
    HEARTBEAT
    ARM
    HEARTBEAT
    START
    stationary TARGET stream at the configured cadence
    STOP
    DISARM
    GET_STATUS
    close

Walking commands, DS4 integration, reconnect policy, sensors, telemetry
aggregation, and autonomous behavior remain outside this milestone.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Protocol

from hexapod.control import MotionCommand
from hexapod.hx1 import (
    UINT32_MAX,
    HX1Ack,
    HX1ClientConfig,
    HX1Event,
    HX1Info,
    HX1Link,
    HX1Nack,
    HX1Outbound,
    HX1Status,
)
from hexapod.motion import MotionPipeline

from .._numeric import float_from_unknown


class RobotRuntimeError(RuntimeError):
    """Base class for Pi runtime orchestration failures."""


class RobotRuntimeTimeoutError(RobotRuntimeError):
    """An expected HX1 response did not arrive within the runtime deadline."""


class RobotRuntimeProtocolError(RobotRuntimeError):
    """The Servo 2040 rejected a command or returned an unexpected response."""


class RobotRuntimePeerStateError(RobotRuntimeError):
    """The Servo 2040 is not in the lifecycle state required by the runtime."""


class RobotRuntimeSafetyError(RobotRuntimeError):
    """The Servo 2040 reported a fault or emergency-stop condition."""


class RuntimeClock(Protocol):
    """Minimal monotonic clock boundary used by the runtime."""

    def monotonic(self) -> float:
        """Return monotonic seconds."""
        ...

    def sleep(self, duration_s: float) -> None:
        """Sleep or advance by ``duration_s``."""
        ...


class SystemRuntimeClock:
    """Production monotonic clock backed by the Python standard library."""

    def monotonic(self) -> float:
        return time.monotonic()

    def sleep(self, duration_s: float) -> None:
        time.sleep(duration_s)


@dataclass(frozen=True, slots=True)
class RobotRuntimeResult:
    """Summary of one successful stationary energized runtime session."""

    staged_joint_vector_deg: tuple[float, ...]
    initial_status: HX1Status
    final_status: HX1Status
    target_frames_sent: int
    heartbeat_frames_sent: int
    requested_active_duration_s: float


def _positive_finite(value: object, name: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{name} must be a finite number > 0")

    try:
        result = float_from_unknown(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be a finite number > 0") from exc

    if not math.isfinite(result) or result <= 0.0:
        raise ValueError(f"{name} must be a finite number > 0")

    return result


class RobotRuntime:
    """Own the minimal Pi-side lifecycle and timing required for HX1 operation.

    The Servo 2040 remains the authoritative hardware-safety state machine.
    This class coordinates the Pi side of that contract and never bypasses the
    MCU's hard position/rate limits, watchdogs, or lifecycle enforcement.

    If an exception or ``KeyboardInterrupt`` occurs after ARM may have reached
    the MCU, the runtime makes a best-effort ESTOP request before closing the
    link. Loss of the link still leaves the MCU watchdog as the final authority.
    """

    def __init__(
        self,
        pipeline: MotionPipeline,
        link: HX1Link,
        config: HX1ClientConfig,
        *,
        clock: RuntimeClock | None = None,
        negotiation_timeout_s: float = 2.0,
        command_timeout_s: float = 0.15,
        poll_interval_s: float = 0.002,
    ):
        if not isinstance(pipeline, MotionPipeline):
            raise TypeError("pipeline must be a MotionPipeline")
        if not isinstance(link, HX1Link):
            raise TypeError("link must be an HX1Link")
        if not isinstance(config, HX1ClientConfig):
            raise TypeError("config must be an HX1ClientConfig")
        if link.client.config != config:
            raise ValueError("runtime config must match the HX1 link client config")

        self.pipeline = pipeline
        self.link = link
        self.config = config
        self.clock = clock if clock is not None else SystemRuntimeClock()

        self.negotiation_timeout_s = _positive_finite(
            negotiation_timeout_s,
            "negotiation_timeout_s",
        )
        self.command_timeout_s = _positive_finite(
            command_timeout_s,
            "command_timeout_s",
        )
        self.poll_interval_s = _positive_finite(
            poll_interval_s,
            "poll_interval_s",
        )

        motion_timeout_s = 0.200
        if self.command_timeout_s >= motion_timeout_s:
            raise ValueError(
                "command_timeout_s must remain below the 200 ms motion watchdog"
            )

        self.last_status: HX1Status | None = None
        self._host_epoch_s: float | None = None
        self._last_heartbeat_s: float | None = None
        self._energization_possible = False
        self._target_frames_sent = 0
        self._heartbeat_frames_sent = 0

    def run_stationary(self, duration_s: float) -> RobotRuntimeResult:
        """Run one complete stationary energized HX1 lifecycle.

        The robot must be mechanically supported for this milestone. Although
        the target remains the configured flat stance, ARM can move unpowered
        servos toward that staged pose because these servos provide no measured
        position feedback.
        """
        duration = _positive_finite(duration_s, "duration_s")

        if self.link.is_open:
            raise RobotRuntimeError("HX1 link must be closed before starting runtime")

        initialization = self.pipeline.initialize_flat_stance()
        staged = initialization.staged_joint_vector_deg

        self.last_status = None
        self._host_epoch_s = self.clock.monotonic()
        self._last_heartbeat_s = None
        self._energization_possible = False
        self._target_frames_sent = 0
        self._heartbeat_frames_sent = 0

        try:
            self.link.open()

            info = self._negotiate()
            if info.state != "DISARMED":
                raise RobotRuntimePeerStateError(
                    f"HELLO peer state must be DISARMED, got {info.state}"
                )

            initial_status = self._request_status()
            self._require_disarmed(initial_status, "before STAGE")

            stage = self.link.client.stage_degrees(staged)
            self._send_and_wait_for_ack(stage)

            self._send_heartbeat()

            arm = self.link.client.arm()

            # Once ARM is sent, the MCU may energize even if its ACK is lost or
            # the host is interrupted before observing that ACK.
            self._energization_possible = True
            self._send_and_wait_for_ack(arm)

            # Refresh immediately after ARM instead of relying solely on the
            # heartbeat accepted while DISARMED.
            self._send_heartbeat()

            start = self.link.client.start()
            self._send_and_wait_for_ack(start)

            active_started_s = self.clock.monotonic()

            self._stream_stationary(
                staged,
                active_started_s=active_started_s,
                duration_s=duration,
            )

            self._normal_shutdown()

            final_status = self._request_status()
            self._require_disarmed(final_status, "after DISARM")

            return RobotRuntimeResult(
                staged_joint_vector_deg=staged,
                initial_status=initial_status,
                final_status=final_status,
                target_frames_sent=self._target_frames_sent,
                heartbeat_frames_sent=self._heartbeat_frames_sent,
                requested_active_duration_s=duration,
            )

        except BaseException:
            if self._energization_possible:
                self._best_effort_estop()
            raise

        finally:
            self.link.close()

    def _negotiate(self) -> HX1Info:
        hello = self.link.client.hello()
        self.link.send(hello)

        deadline = self.clock.monotonic() + self.negotiation_timeout_s

        while self.clock.monotonic() < deadline:
            for message in self._poll_messages():
                if isinstance(message, HX1Info) and message.ref_seq == hello.seq:
                    return message

            self.clock.sleep(self.poll_interval_s)

        raise RobotRuntimeTimeoutError("timed out waiting for HELLO/INFO")

    def _request_status(self) -> HX1Status:
        request = self.link.client.get_status()
        self.link.send(request)

        session = self.link.client.session
        if session is None:
            raise RobotRuntimeProtocolError(
                "GET_STATUS requires a negotiated HX1 session"
            )

        deadline = self.clock.monotonic() + self.command_timeout_s

        while self.clock.monotonic() < deadline:
            for message in self._poll_messages():
                if isinstance(message, HX1Status) and message.session == session:
                    return message

            self.clock.sleep(self.poll_interval_s)

        raise RobotRuntimeTimeoutError("timed out waiting for GET_STATUS/STATUS")

    def _send_and_wait_for_ack(self, outbound: HX1Outbound) -> HX1Ack:
        self.link.send(outbound)

        deadline = self.clock.monotonic() + self.command_timeout_s

        while self.clock.monotonic() < deadline:
            for message in self._poll_messages():
                if (
                    isinstance(message, HX1Ack)
                    and message.ref_seq == outbound.seq
                    and message.command == outbound.command
                ):
                    return message

            self.clock.sleep(self.poll_interval_s)

        raise RobotRuntimeTimeoutError(f"timed out waiting for {outbound.command} ACK")

    def _poll_messages(self):
        messages = self.link.poll()

        for message in messages:
            self._observe_message(message)

        return messages

    def _observe_message(self, message) -> None:
        if isinstance(message, HX1Nack):
            raise RobotRuntimeProtocolError(
                f"{message.command} seq={message.ref_seq} rejected: {message.error}"
            )

        if isinstance(message, HX1Event):
            if message.event_type in {"FAULT", "ESTOP"}:
                raise RobotRuntimeSafetyError(
                    f"MCU event {message.event_type}: {message.detail}"
                )
            return

        if isinstance(message, HX1Status):
            self.last_status = message

            if message.state in {"FAULT", "ESTOP"} or message.fault != "NONE":
                raise RobotRuntimeSafetyError(
                    f"MCU status unsafe: state={message.state} fault={message.fault}"
                )

    def _require_disarmed(
        self,
        status: HX1Status,
        context: str,
    ) -> None:
        if status.state != "DISARMED" or status.fault != "NONE":
            raise RobotRuntimePeerStateError(
                f"MCU must be DISARMED/NONE {context}: "
                f"state={status.state} fault={status.fault}"
            )

    def _send_heartbeat(self) -> None:
        now = self.clock.monotonic()
        heartbeat = self.link.client.heartbeat(
            self._host_uptime_ms(now),
        )
        self.link.send(heartbeat)

        self._heartbeat_frames_sent += 1
        self._last_heartbeat_s = now

    def _host_uptime_ms(self, now_s: float) -> int:
        if self._host_epoch_s is None:
            raise RobotRuntimeError("runtime host epoch is not initialized")

        elapsed_s = max(0.0, now_s - self._host_epoch_s)
        return int(elapsed_s * 1000.0) & UINT32_MAX

    def _stream_stationary(
        self,
        staged_joint_vector_deg: tuple[float, ...],
        *,
        active_started_s: float,
        duration_s: float,
    ) -> None:
        target_period_s = self.config.target_period_ms / 1000.0
        heartbeat_period_s = self.config.heartbeat_period_ms / 1000.0

        end_s = active_started_s + duration_s
        next_target_s = active_started_s + target_period_s

        if self._last_heartbeat_s is None:
            next_heartbeat_s = active_started_s
        else:
            next_heartbeat_s = self._last_heartbeat_s + heartbeat_period_s

        last_pipeline_s = active_started_s

        while True:
            self._poll_messages()
            now = self.clock.monotonic()

            if now >= end_s:
                return

            if now >= next_heartbeat_s:
                self._send_heartbeat()
                now = self.clock.monotonic()

                while next_heartbeat_s <= now:
                    next_heartbeat_s += heartbeat_period_s

            if now >= next_target_s:
                frame = self.pipeline.step(
                    MotionCommand.zero(),
                    max(0.0, now - last_pipeline_s),
                )

                # Milestone 2 must remain exactly stationary. Keep this
                # invariant explicit even though a zero command should already
                # produce the seeded flat stance.
                if frame.output_joint_vector_deg != staged_joint_vector_deg:
                    raise RobotRuntimeSafetyError(
                        "stationary pipeline output diverged from staged flat stance"
                    )

                target = self.link.client.target_degrees(frame.output_joint_vector_deg)
                self.link.send(target)

                self._target_frames_sent += 1
                last_pipeline_s = now

                while next_target_s <= now:
                    next_target_s += target_period_s

            next_deadline = min(
                end_s,
                next_target_s,
                next_heartbeat_s,
            )
            sleep_s = min(
                self.poll_interval_s,
                max(0.0, next_deadline - self.clock.monotonic()),
            )

            if sleep_s > 0.0:
                self.clock.sleep(sleep_s)

    def _normal_shutdown(self) -> None:
        stop = self.link.client.stop()
        self._send_and_wait_for_ack(stop)

        disarm = self.link.client.disarm()
        self._send_and_wait_for_ack(disarm)

        # A matching DISARM ACK confirms the MCU accepted the transition to its
        # non-energized normal resting state.
        self._energization_possible = False

    def _best_effort_estop(self) -> None:
        if not self.link.is_open:
            return

        try:
            estop = self.link.client.estop("pi_runtime_failure")
            self.link.send(estop)
        except BaseException:
            # The Servo 2040 link watchdog remains the final fail-safe when
            # the host can no longer transmit a valid emergency-stop frame.
            pass
