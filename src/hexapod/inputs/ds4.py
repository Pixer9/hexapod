"""Pure DualShock 4 mapping and state logic.

This module has no dependency on Linux evdev. Raw Linux input events are
adapted by ``ds4_evdev.py`` and fed into the pure state/mapping objects here.

The configured default axis signs have been physically verified on the
target Raspberry Pi 5 / DualShock 4 combination:

    vx       <- -ABS_Y
    vy       <- -ABS_X
    yaw_rate <- -ABS_RX

The axis bindings remain configuration rather than mathematical robot
invariants, but these signs are the verified baseline for this hardware.
"""

from __future__ import annotations

import json
import math
from collections import deque
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from types import MappingProxyType
from typing import Mapping

from hexapod.control import CommandSample, MotionCommand, MotionLimits, MotionSource


class DS4ConfigError(ValueError):
    """DS4 configuration is malformed or unsupported."""


@dataclass(frozen=True, slots=True)
class AxisBinding:
    code: str
    invert: bool


@dataclass(frozen=True, slots=True)
class DS4Config:
    schema_version: int
    adapter_id: str
    device_path: str | None
    device_name_contains: tuple[str, ...]
    deadzone: float
    expo: float
    command_ttl_s: float
    start_motion_enabled: bool
    axes: Mapping[str, AxisBinding]
    button_toggle_motion: str
    button_estop: str
    button_clear_estop_request: str

    def __post_init__(self) -> None:
        if not isinstance(self.axes, MappingProxyType):
            object.__setattr__(
                self,
                "axes",
                MappingProxyType(dict(self.axes)),
            )


class DS4ActionType(str, Enum):
    MOTION_ENABLED = "motion_enabled"
    MOTION_DISABLED = "motion_disabled"
    ESTOP = "estop"
    CLEAR_ESTOP_REQUEST = "clear_estop_request"
    DISCONNECTED = "disconnected"


@dataclass(frozen=True, slots=True)
class DS4Action:
    action: DS4ActionType
    occurred_at_s: float


@dataclass(frozen=True, slots=True)
class DS4Snapshot:
    connected: bool
    motion_enabled: bool
    command: MotionCommand
    last_event_at_s: float | None
    device_path: str | None
    device_name: str | None


def _finite_float(value: object, name: str) -> float:
    if isinstance(value, bool):
        raise DS4ConfigError(f"{name} must be a finite number")

    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise DS4ConfigError(f"{name} must be a finite number") from exc

    if not math.isfinite(result):
        raise DS4ConfigError(f"{name} must be a finite number")

    return result


def _nonempty_string(value: object, name: str) -> str:
    if not isinstance(value, str) or not value:
        raise DS4ConfigError(f"{name} must be a non-empty string")
    return value


def _require_dict(value: object, name: str) -> dict:
    if not isinstance(value, dict):
        raise DS4ConfigError(f"{name} must be an object")
    return value


def _load_binding(value: object, name: str) -> AxisBinding:
    item = _require_dict(value, name)
    code = _nonempty_string(item.get("code"), f"{name}.code")
    invert = item.get("invert")
    if not isinstance(invert, bool):
        raise DS4ConfigError(f"{name}.invert must be boolean")
    return AxisBinding(code=code, invert=invert)


def parse_ds4_config(data: object) -> DS4Config:
    root = _require_dict(data, "root")

    if root.get("schema_version") != 1:
        raise DS4ConfigError("schema_version must be 1")

    adapter_id = _nonempty_string(root.get("adapter_id"), "adapter_id")

    device = _require_dict(root.get("device"), "device")
    device_path = device.get("path")
    if device_path is not None:
        device_path = _nonempty_string(device_path, "device.path")

    names_raw = device.get("name_contains")
    if not isinstance(names_raw, list) or not names_raw:
        raise DS4ConfigError("device.name_contains must be a non-empty array")
    device_names = tuple(
        _nonempty_string(value, "device.name_contains[]") for value in names_raw
    )

    axis_processing = _require_dict(
        root.get("axis_processing"),
        "axis_processing",
    )
    deadzone = _finite_float(
        axis_processing.get("deadzone"),
        "axis_processing.deadzone",
    )
    expo = _finite_float(
        axis_processing.get("expo"),
        "axis_processing.expo",
    )

    if not 0.0 <= deadzone < 1.0:
        raise DS4ConfigError("axis_processing.deadzone must be in [0, 1)")
    if not 0.0 <= expo <= 1.0:
        raise DS4ConfigError("axis_processing.expo must be in [0, 1]")

    command_ttl_s = _finite_float(
        root.get("command_ttl_s"),
        "command_ttl_s",
    )
    if command_ttl_s <= 0.0:
        raise DS4ConfigError("command_ttl_s must be > 0")

    start_enabled = root.get("start_motion_enabled")
    if not isinstance(start_enabled, bool):
        raise DS4ConfigError("start_motion_enabled must be boolean")

    axes_raw = _require_dict(root.get("axes"), "axes")
    if set(axes_raw) != {"vx", "vy", "yaw_rate"}:
        raise DS4ConfigError("axes must contain exactly vx, vy, and yaw_rate")
    axes = {
        key: _load_binding(axes_raw[key], f"axes.{key}")
        for key in ("vx", "vy", "yaw_rate")
    }

    codes = [binding.code for binding in axes.values()]
    if len(set(codes)) != len(codes):
        raise DS4ConfigError("vx, vy, and yaw_rate must use distinct axes")

    buttons = _require_dict(root.get("buttons"), "buttons")

    return DS4Config(
        schema_version=1,
        adapter_id=adapter_id,
        device_path=device_path,
        device_name_contains=device_names,
        deadzone=deadzone,
        expo=expo,
        command_ttl_s=command_ttl_s,
        start_motion_enabled=start_enabled,
        axes=axes,
        button_toggle_motion=_nonempty_string(
            buttons.get("toggle_motion_enabled"),
            "buttons.toggle_motion_enabled",
        ),
        button_estop=_nonempty_string(
            buttons.get("estop"),
            "buttons.estop",
        ),
        button_clear_estop_request=_nonempty_string(
            buttons.get("clear_estop_request"),
            "buttons.clear_estop_request",
        ),
    )


def load_ds4_config(path: str | Path) -> DS4Config:
    path = Path(path)

    try:
        text = path.read_text(encoding="utf-8")
    except OSError as exc:
        raise DS4ConfigError(f"could not read DS4 config {path}: {exc}") from exc

    try:
        data = json.loads(text)
    except json.JSONDecodeError as exc:
        raise DS4ConfigError(f"invalid JSON in DS4 config {path}: {exc}") from exc

    return parse_ds4_config(data)


def normalize_axis(value: int, minimum: int, maximum: int) -> float:
    """Normalize one evdev ABS value to [-1, +1]."""
    if maximum <= minimum:
        raise ValueError("axis maximum must be greater than minimum")

    normalized = (
        (float(value) - float(minimum)) / float(maximum - minimum)
    ) * 2.0 - 1.0

    return max(-1.0, min(1.0, normalized))


def _shape_axis(value: float, deadzone: float, expo: float) -> float:
    value = max(-1.0, min(1.0, float(value)))

    if abs(value) <= deadzone:
        return 0.0

    magnitude = (abs(value) - deadzone) / (1.0 - deadzone)
    value = math.copysign(magnitude, value)

    return (1.0 - expo) * value + expo * (value**3)


class DS4Mapper:
    """Convert normalized configured DS4 axes into ``MotionCommand``."""

    def __init__(self, config: DS4Config, limits: MotionLimits):
        self.config = config
        self.limits = limits

    def command_from_axes(
        self,
        normalized_axes: Mapping[str, float],
    ) -> MotionCommand:
        values: dict[str, float] = {}

        for output_name in ("vx", "vy", "yaw_rate"):
            binding = self.config.axes[output_name]
            raw = float(normalized_axes.get(binding.code, 0.0))
            shaped = _shape_axis(
                raw,
                self.config.deadzone,
                self.config.expo,
            )
            if binding.invert:
                shaped = -shaped
            values[output_name] = shaped

        command = MotionCommand(
            vx_mm_s=values["vx"] * self.limits.max_vx_mm_s,
            vy_mm_s=values["vy"] * self.limits.max_vy_mm_s,
            yaw_rate_deg_s=(values["yaw_rate"] * self.limits.max_yaw_rate_deg_s),
        )

        return self.limits.validate(command)


class DS4InputState:
    """Thread-safe-wrapper-friendly pure controller state.

    The evdev adapter owns synchronization. This class deliberately contains no
    locks and no hardware imports so its behavior can be tested deterministically.
    """

    def __init__(self, config: DS4Config, limits: MotionLimits):
        self.config = config
        self.mapper = DS4Mapper(config, limits)
        self.normalized_axes: dict[str, float] = {
            binding.code: 0.0 for binding in config.axes.values()
        }
        self.connected = False
        self.motion_enabled = config.start_motion_enabled
        self.last_event_at_s: float | None = None
        self.device_path: str | None = None
        self.device_name: str | None = None
        self._actions: deque[DS4Action] = deque()

    def connect(
        self,
        *,
        device_path: str,
        device_name: str,
        now_s: float,
    ) -> None:
        self.connected = True
        self.device_path = device_path
        self.device_name = device_name
        self.last_event_at_s = float(now_s)

    def update_axis(
        self,
        code: str,
        normalized_value: float,
        *,
        now_s: float,
    ) -> None:
        if code not in self.normalized_axes:
            return

        value = float(normalized_value)
        if not math.isfinite(value):
            raise ValueError("normalized axis value must be finite")

        self.normalized_axes[code] = max(-1.0, min(1.0, value))
        self.last_event_at_s = float(now_s)

    def button_pressed(self, code: str, *, now_s: float) -> None:
        now = float(now_s)
        self.last_event_at_s = now

        if code == self.config.button_toggle_motion:
            self.motion_enabled = not self.motion_enabled
            self._actions.append(
                DS4Action(
                    DS4ActionType.MOTION_ENABLED
                    if self.motion_enabled
                    else DS4ActionType.MOTION_DISABLED,
                    now,
                )
            )
            return

        if code == self.config.button_estop:
            self._actions.append(DS4Action(DS4ActionType.ESTOP, now))
            return

        if code == self.config.button_clear_estop_request:
            self._actions.append(DS4Action(DS4ActionType.CLEAR_ESTOP_REQUEST, now))

    def disconnect(self, *, now_s: float) -> None:
        if self.connected:
            self._actions.append(DS4Action(DS4ActionType.DISCONNECTED, float(now_s)))
            # A disconnect is also an explicit safety request. The future
            # safety supervisor decides how that request maps into runtime state.
            self._actions.append(DS4Action(DS4ActionType.ESTOP, float(now_s)))

        self.connected = False
        self.motion_enabled = False
        self.last_event_at_s = float(now_s)

    def snapshot(self) -> DS4Snapshot:
        command = self.mapper.command_from_axes(self.normalized_axes)

        if not self.motion_enabled:
            command = MotionCommand.zero()

        return DS4Snapshot(
            connected=self.connected,
            motion_enabled=self.motion_enabled,
            command=command,
            last_event_at_s=self.last_event_at_s,
            device_path=self.device_path,
            device_name=self.device_name,
        )

    def command_sample(self, *, now_s: float) -> CommandSample | None:
        """Publish current stick state while the controller connection is alive.

        Freshness is intentionally based on *publication time*, not the time of
        the last evdev axis event. A held stick does not continuously generate
        input events, but it remains a valid operator command while the device
        is connected.
        """
        if not self.connected:
            return None

        snapshot = self.snapshot()
        now = float(now_s)

        return CommandSample(
            source=MotionSource.DS4,
            command=snapshot.command,
            received_at_s=now,
            expires_at_s=now + self.config.command_ttl_s,
        )

    def drain_actions(self) -> tuple[DS4Action, ...]:
        actions = tuple(self._actions)
        self._actions.clear()
        return actions
