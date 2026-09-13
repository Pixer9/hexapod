"""Freshness-aware arbitration between robot motion-command sources.

Emergency stop is intentionally not modeled as another motion-command source.
The safety supervisor sits above this arbiter and can suppress/disarm motion
regardless of which normal source would otherwise win.

Normal source priority is fixed:

    DS4 > WEB > AUTONOMY > IDLE

The priority itself is a safety/control contract rather than a tunable runtime
configuration value.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math

from .command import MotionCommand, MotionLimits


class MotionSource(str, Enum):
    IDLE = "idle"
    AUTONOMY = "autonomy"
    WEB = "web"
    DS4 = "ds4"


_SOURCE_PRIORITY = {
    MotionSource.IDLE: 0,
    MotionSource.AUTONOMY: 10,
    MotionSource.WEB: 20,
    MotionSource.DS4: 30,
}


def _finite_time(value: object, name: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{name} must be a finite monotonic timestamp")

    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be a finite monotonic timestamp") from exc

    if not math.isfinite(result):
        raise ValueError(f"{name} must be a finite monotonic timestamp")

    return result


@dataclass(frozen=True, slots=True)
class CommandSample:
    """One time-bounded command published by a normal control source."""

    source: MotionSource
    command: MotionCommand
    received_at_s: float
    expires_at_s: float

    def __post_init__(self) -> None:
        if not isinstance(self.source, MotionSource):
            raise ValueError("source must be a MotionSource")

        if self.source is MotionSource.IDLE:
            raise ValueError("IDLE is synthesized by the arbiter, not published")

        if not isinstance(self.command, MotionCommand):
            raise ValueError("command must be a MotionCommand")

        received = _finite_time(self.received_at_s, "received_at_s")
        expires = _finite_time(self.expires_at_s, "expires_at_s")

        if expires <= received:
            raise ValueError("expires_at_s must be greater than received_at_s")

        object.__setattr__(self, "received_at_s", received)
        object.__setattr__(self, "expires_at_s", expires)

    def is_fresh(self, now_s: float) -> bool:
        now = _finite_time(now_s, "now_s")
        return self.received_at_s <= now < self.expires_at_s


@dataclass(frozen=True, slots=True)
class SelectedMotionCommand:
    source: MotionSource
    command: MotionCommand
    age_s: float

    @property
    def is_idle(self) -> bool:
        return self.source is MotionSource.IDLE


class CommandArbiter:
    """Select the highest-priority currently fresh normal command source."""

    def __init__(self, limits: MotionLimits):
        self._limits = limits
        self._latest: dict[MotionSource, CommandSample] = {}

    def publish(self, sample: CommandSample) -> None:
        # Reject out-of-envelope commands at the common boundary so every
        # source receives the same physical operating limits.
        self._limits.validate(sample.command)

        previous = self._latest.get(sample.source)
        if previous is not None and sample.received_at_s < previous.received_at_s:
            raise ValueError(f"stale {sample.source.value} command publication")

        self._latest[sample.source] = sample

    def clear(self, source: MotionSource) -> None:
        if source is MotionSource.IDLE:
            return
        self._latest.pop(source, None)

    def select(self, now_s: float) -> SelectedMotionCommand:
        now = _finite_time(now_s, "now_s")
        candidates = [
            sample for sample in self._latest.values() if sample.is_fresh(now)
        ]

        if not candidates:
            return SelectedMotionCommand(
                source=MotionSource.IDLE,
                command=MotionCommand.zero(),
                age_s=0.0,
            )

        winner = max(
            candidates,
            key=lambda sample: _SOURCE_PRIORITY[sample.source],
        )

        return SelectedMotionCommand(
            source=winner.source,
            command=winner.command,
            age_s=now - winner.received_at_s,
        )
