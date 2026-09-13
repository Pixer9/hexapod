"""Linux evdev adapter for the pure DS4 input model.

``evdev`` is imported lazily so host-side tests and non-DS4 tools do not require
the package merely to import ``hexapod.inputs``.
"""

from __future__ import annotations

from dataclasses import dataclass
import logging
import threading
import time
from typing import Any

from .ds4 import (
    DS4Config,
    DS4InputState,
    normalize_axis,
)
from hexapod.control import CommandSample, MotionLimits


log = logging.getLogger(__name__)


class DS4EvdevError(RuntimeError):
    """Linux evdev DS4 initialization or runtime failure."""


@dataclass(frozen=True, slots=True)
class DS4DeviceInfo:
    path: str
    name: str


def _load_evdev():
    try:
        import evdev
    except ImportError as exc:
        raise DS4EvdevError(
            "DS4 support requires the 'evdev' package in the project venv"
        ) from exc
    return evdev


def _resolve_code(ecodes: Any, name: str) -> int:
    value = getattr(ecodes, name, None)
    if not isinstance(value, int):
        raise DS4EvdevError(f"evdev has no input code named {name}")
    return value


def _device_matches(
    device: Any,
    config: DS4Config,
    evdev_module: Any,
) -> bool:
    name = str(getattr(device, "name", "") or "")
    if not any(
        fragment.lower() in name.lower() for fragment in config.device_name_contains
    ):
        return False

    try:
        capabilities = device.capabilities(absinfo=False)
    except Exception:
        return False

    abs_codes = set(capabilities.get(evdev_module.ecodes.EV_ABS, ()))

    for binding in config.axes.values():
        try:
            code = _resolve_code(evdev_module.ecodes, binding.code)
        except DS4EvdevError:
            return False
        if code not in abs_codes:
            return False

    return True


def discover_ds4_device(
    config: DS4Config,
    *,
    evdev_module: Any | None = None,
) -> DS4DeviceInfo:
    """Resolve the configured DS4 input node.

    When ``device.path`` is set, that exact node is used. Otherwise all evdev
    nodes are scanned and the first name/capability match is selected.
    """
    evdev = evdev_module if evdev_module is not None else _load_evdev()

    if config.device_path is not None:
        try:
            device = evdev.InputDevice(config.device_path)
        except Exception as exc:
            raise DS4EvdevError(
                f"could not open configured DS4 device {config.device_path}: {exc}"
            ) from exc

        try:
            if not _device_matches(device, config, evdev):
                raise DS4EvdevError(
                    f"configured device {config.device_path} does not match "
                    "the DS4 name/axis requirements"
                )
            return DS4DeviceInfo(
                path=str(device.path),
                name=str(device.name),
            )
        finally:
            try:
                device.close()
            except Exception:
                pass

    matches: list[DS4DeviceInfo] = []

    for path in evdev.list_devices():
        try:
            device = evdev.InputDevice(path)
        except Exception:
            continue

        try:
            if _device_matches(device, config, evdev):
                matches.append(
                    DS4DeviceInfo(
                        path=str(device.path),
                        name=str(device.name),
                    )
                )
        finally:
            try:
                device.close()
            except Exception:
                pass

    if not matches:
        raise DS4EvdevError("no matching DS4 evdev node was found")

    # Stable ordering avoids node-enumeration order deciding between otherwise
    # equivalent candidates.
    matches.sort(key=lambda item: item.path)
    return matches[0]


class DS4EvdevReader:
    """Background evdev reader that exposes current DS4 intent snapshots."""

    def __init__(
        self,
        config: DS4Config,
        limits: MotionLimits,
        *,
        clock=time.monotonic,
        evdev_module: Any | None = None,
    ):
        self.config = config
        self._clock = clock
        self._evdev = evdev_module if evdev_module is not None else _load_evdev()
        self._state = DS4InputState(config, limits)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._device: Any | None = None
        self._axis_ranges: dict[int, tuple[int, int]] = {}
        self._axis_names: dict[int, str] = {}

    def start(self) -> DS4DeviceInfo:
        if self._thread is not None and self._thread.is_alive():
            raise DS4EvdevError("DS4 reader is already running")

        info = discover_ds4_device(
            self.config,
            evdev_module=self._evdev,
        )

        try:
            device = self._evdev.InputDevice(info.path)
        except Exception as exc:
            raise DS4EvdevError(
                f"could not open DS4 device {info.path}: {exc}"
            ) from exc

        self._device = device
        self._stop.clear()
        self._axis_ranges = {}
        self._axis_names = {}

        now = self._clock()

        with self._lock:
            self._state = DS4InputState(
                self.config,
                self._state.mapper.limits,
            )
            self._state.connect(
                device_path=info.path,
                device_name=info.name,
                now_s=now,
            )

            # Seed every mapped axis from the device's current ABS value so
            # starting the reader does not assume the sticks are centered.
            for binding in self.config.axes.values():
                code = _resolve_code(
                    self._evdev.ecodes,
                    binding.code,
                )
                try:
                    absinfo = device.absinfo(code)
                except Exception as exc:
                    device.close()
                    self._device = None
                    raise DS4EvdevError(
                        f"could not read {binding.code} ABS info: {exc}"
                    ) from exc

                minimum = int(absinfo.min)
                maximum = int(absinfo.max)
                self._axis_ranges[code] = (minimum, maximum)
                self._axis_names[code] = binding.code

                self._state.update_axis(
                    binding.code,
                    normalize_axis(
                        int(absinfo.value),
                        minimum,
                        maximum,
                    ),
                    now_s=now,
                )

        self._thread = threading.Thread(
            target=self._run,
            name="hexapod-ds4",
            daemon=True,
        )
        self._thread.start()
        return info

    def stop(self) -> None:
        self._stop.set()

        device = self._device
        if device is not None:
            try:
                device.close()
            except Exception:
                pass

        thread = self._thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=1.0)

        with self._lock:
            if self._state.connected:
                # Intentional shutdown should stop publication but should not
                # manufacture a DISCONNECTED/ESTOP action.
                self._state.connected = False
                self._state.motion_enabled = False

        self._device = None

    def snapshot(self):
        with self._lock:
            return self._state.snapshot()

    def command_sample(self, *, now_s: float | None = None) -> CommandSample | None:
        now = self._clock() if now_s is None else float(now_s)
        with self._lock:
            return self._state.command_sample(now_s=now)

    def drain_actions(self):
        with self._lock:
            return self._state.drain_actions()

    def _handle_event(self, event: Any, *, now_s: float) -> None:
        ecodes = self._evdev.ecodes

        if event.type == ecodes.EV_ABS:
            code = int(event.code)
            if code not in self._axis_ranges:
                return

            minimum, maximum = self._axis_ranges[code]
            axis_name = self._axis_names[code]

            with self._lock:
                self._state.update_axis(
                    axis_name,
                    normalize_axis(
                        int(event.value),
                        minimum,
                        maximum,
                    ),
                    now_s=now_s,
                )
            return

        if event.type != ecodes.EV_KEY or int(event.value) != 1:
            return

        button_by_code = {
            _resolve_code(
                ecodes, self.config.button_toggle_motion
            ): self.config.button_toggle_motion,
            _resolve_code(ecodes, self.config.button_estop): self.config.button_estop,
            _resolve_code(
                ecodes, self.config.button_clear_estop_request
            ): self.config.button_clear_estop_request,
        }

        button_name = button_by_code.get(int(event.code))
        if button_name is None:
            return

        with self._lock:
            self._state.button_pressed(
                button_name,
                now_s=now_s,
            )

    def _run(self) -> None:
        device = self._device
        if device is None:
            return

        disconnected = False

        try:
            for event in device.read_loop():
                if self._stop.is_set():
                    return
                self._handle_event(
                    event,
                    now_s=self._clock(),
                )

            # A controller reader that ends on its own is no longer a live
            # command source, even if evdev did not raise an exception.
            if not self._stop.is_set():
                disconnected = True
                log.warning("DS4 evdev read loop ended unexpectedly")

        except OSError:
            if not self._stop.is_set():
                disconnected = True
                log.warning("DS4 evdev device disconnected")

        except Exception:
            if not self._stop.is_set():
                disconnected = True
                log.exception("DS4 evdev reader failed")

        finally:
            if disconnected:
                with self._lock:
                    self._state.disconnect(
                        now_s=self._clock(),
                    )
