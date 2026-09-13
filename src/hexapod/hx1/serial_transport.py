"""PySerial-backed HX1 raw-byte transport for Raspberry Pi Linux.

The module imports pyserial lazily so pure protocol, fake-transport, and host
tests do not require pyserial merely to import ``hexapod.hx1``.

This adapter owns only the physical serial byte stream. It does not frame HX1
messages, negotiate sessions, schedule heartbeats/targets, reconnect
automatically, or make robot safety decisions.
"""

from __future__ import annotations

from collections.abc import Callable
from typing import Any

from .config import HX1ClientConfig
from .transport import (
    HX1TransportClosedError,
    HX1TransportError,
    HX1TransportValueError,
    _normalize_write_data,
    _validate_max_bytes,
)


class HX1SerialDependencyError(HX1TransportError):
    """The production serial adapter requires pyserial in the project venv."""


SerialFactory = Callable[..., Any]


def _load_serial_factory() -> SerialFactory:
    try:
        import serial
    except ImportError as exc:
        raise HX1SerialDependencyError(
            "SerialHX1Transport requires the 'pyserial' package "
            "in the project venv"
        ) from exc

    return serial.Serial


class SerialHX1Transport:
    """Non-blocking pyserial implementation of the HX1 transport contract."""

    def __init__(
        self,
        device_path: str,
        baudrate: int,
        *,
        write_timeout_s: float,
        serial_factory: SerialFactory | None = None,
    ):
        if not isinstance(device_path, str) or not device_path:
            raise HX1TransportValueError(
                "device_path must be a non-empty string"
            )
        if (
            isinstance(baudrate, bool)
            or not isinstance(baudrate, int)
            or baudrate <= 0
        ):
            raise HX1TransportValueError(
                "baudrate must be a positive integer"
            )
        if isinstance(write_timeout_s, bool):
            raise HX1TransportValueError(
                "write_timeout_s must be a finite number > 0"
            )
        try:
            timeout = float(write_timeout_s)
        except (TypeError, ValueError) as exc:
            raise HX1TransportValueError(
                "write_timeout_s must be a finite number > 0"
            ) from exc
        if not 0.0 < timeout < float("inf"):
            raise HX1TransportValueError(
                "write_timeout_s must be a finite number > 0"
            )

        self.device_path = device_path
        self.baudrate = baudrate
        self.write_timeout_s = timeout
        self._serial_factory = serial_factory
        self._serial: Any | None = None

    @property
    def is_open(self) -> bool:
        serial_port = self._serial
        return bool(
            serial_port is not None
            and getattr(serial_port, "is_open", False)
        )

    def open(self) -> None:
        if self.is_open:
            return

        factory = (
            self._serial_factory
            if self._serial_factory is not None
            else _load_serial_factory()
        )

        try:
            serial_port = factory(
                port=self.device_path,
                baudrate=self.baudrate,
                timeout=0,
                write_timeout=self.write_timeout_s,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False,
                exclusive=True,
            )
        except Exception as exc:
            self._serial = None
            raise HX1TransportError(
                f"could not open HX1 serial device "
                f"{self.device_path}: {exc}"
            ) from exc

        if not getattr(serial_port, "is_open", False):
            try:
                serial_port.close()
            except Exception:
                pass
            self._serial = None
            raise HX1TransportError(
                f"HX1 serial device {self.device_path} "
                "did not open"
            )

        self._serial = serial_port

    def close(self) -> None:
        serial_port = self._serial
        if serial_port is None:
            return

        # Clear local availability before touching the backend. Even if close
        # fails, callers must not keep treating this object as a usable link.
        self._serial = None

        try:
            serial_port.close()
        except Exception as exc:
            raise HX1TransportError(
                f"could not close HX1 serial device "
                f"{self.device_path}: {exc}"
            ) from exc

    def read(self, max_bytes: int = 4096) -> bytes:
        serial_port = self._require_open()
        count = _validate_max_bytes(max_bytes)

        try:
            waiting = int(serial_port.in_waiting)
        except Exception as exc:
            raise HX1TransportError(
                f"could not query HX1 serial input: {exc}"
            ) from exc

        if waiting <= 0:
            return b""

        request = min(count, waiting)

        try:
            data = serial_port.read(request)
        except Exception as exc:
            raise HX1TransportError(
                f"HX1 serial read failed: {exc}"
            ) from exc

        if not isinstance(data, (bytes, bytearray)):
            raise HX1TransportError(
                "HX1 serial read returned non-byte data"
            )

        result = bytes(data)
        if len(result) > request:
            raise HX1TransportError(
                "HX1 serial read returned more bytes than requested"
            )
        return result

    def write(self, data: bytes | bytearray) -> int:
        serial_port = self._require_open()
        payload = _normalize_write_data(data)

        if not payload:
            return 0

        offset = 0
        while offset < len(payload):
            try:
                written = serial_port.write(payload[offset:])
            except Exception as exc:
                raise HX1TransportError(
                    f"HX1 serial write failed: {exc}"
                ) from exc

            if (
                isinstance(written, bool)
                or not isinstance(written, int)
                or written <= 0
            ):
                raise HX1TransportError(
                    "HX1 serial write made no forward progress"
                )

            remaining = len(payload) - offset
            if written > remaining:
                raise HX1TransportError(
                    "HX1 serial backend reported an invalid write count"
                )

            offset += written

        return offset

    def _require_open(self) -> Any:
        if not self.is_open:
            raise HX1TransportClosedError(
                "HX1 serial transport is not open"
            )
        return self._serial


def create_serial_hx1_transport(
    config: HX1ClientConfig,
    *,
    serial_factory: SerialFactory | None = None,
) -> SerialHX1Transport:
    """Construct the physical serial adapter from the HX1 client config."""
    if not isinstance(config, HX1ClientConfig):
        raise HX1TransportValueError(
            "config must be an HX1ClientConfig"
        )
    if config.transport_kind != "usb_cdc_serial":
        raise HX1TransportValueError(
            "HX1 config transport kind is not usb_cdc_serial"
        )

    return SerialHX1Transport(
        config.device_path,
        config.baudrate,
        write_timeout_s=config.write_timeout_s,
        serial_factory=serial_factory,
    )
