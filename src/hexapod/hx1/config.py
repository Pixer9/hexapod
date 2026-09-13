"""Strict configuration for the Pi-side HX1 peer policy."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path

from .protocol import PROTOCOL_MINOR


class HX1ConfigError(ValueError):
    """HX1 client configuration is malformed or unsupported."""


@dataclass(frozen=True, slots=True)
class HX1ClientConfig:
    schema_version: int
    link_id: str
    transport_kind: str
    device_path: str
    baudrate: int
    write_timeout_s: float
    client_minor: int
    required_server_minor: int
    joint_count: int
    heartbeat_period_ms: int
    target_period_ms: int
    expected_profile_id: str
    expected_profile_revision: int
    expected_profile_sha256: str


def _nonempty_string(value: object, name: str) -> str:
    if not isinstance(value, str) or not value:
        raise HX1ConfigError(f"{name} must be a non-empty string")
    return value


def _positive_int(value: object, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise HX1ConfigError(f"{name} must be a positive integer")
    return value


def _nonnegative_int(value: object, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise HX1ConfigError(
            f"{name} must be a non-negative integer"
        )
    return value


def _positive_float(value: object, name: str) -> float:
    if isinstance(value, bool):
        raise HX1ConfigError(f"{name} must be a finite number > 0")
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise HX1ConfigError(
            f"{name} must be a finite number > 0"
        ) from exc
    if not math.isfinite(result) or result <= 0.0:
        raise HX1ConfigError(f"{name} must be a finite number > 0")
    return result


def _require_exact_keys(
    data: dict,
    expected: set[str],
    name: str,
) -> None:
    missing = expected - set(data)
    extra = set(data) - expected
    if missing:
        raise HX1ConfigError(
            f"{name} missing keys: " + ", ".join(sorted(missing))
        )
    if extra:
        raise HX1ConfigError(
            f"{name} unsupported keys: " + ", ".join(sorted(extra))
        )


def _sha256_text(value: object, name: str) -> str:
    text = _nonempty_string(value, name)
    if len(text) != 64 or any(
        not ("0" <= c <= "9" or "A" <= c <= "F")
        for c in text
    ):
        raise HX1ConfigError(
            f"{name} must contain exactly 64 uppercase hex digits"
        )
    return text


def parse_hx1_client_config(data: object) -> HX1ClientConfig:
    if not isinstance(data, dict):
        raise HX1ConfigError("root must be an object")

    _require_exact_keys(
        data,
        {
            "schema_version",
            "link_id",
            "transport",
            "protocol",
            "expected_profile",
        },
        "root",
    )

    if data["schema_version"] != 1:
        raise HX1ConfigError("schema_version must be 1")

    transport = data["transport"]
    protocol = data["protocol"]
    expected_profile = data["expected_profile"]

    if not isinstance(transport, dict):
        raise HX1ConfigError("transport must be an object")
    if not isinstance(protocol, dict):
        raise HX1ConfigError("protocol must be an object")
    if not isinstance(expected_profile, dict):
        raise HX1ConfigError(
            "expected_profile must be an object"
        )

    _require_exact_keys(
        transport,
        {
            "kind",
            "device",
            "baudrate",
            "write_timeout_s",
        },
        "transport",
    )
    _require_exact_keys(
        protocol,
        {
            "client_minor",
            "required_server_minor",
            "joint_count",
            "heartbeat_period_ms",
            "target_period_ms",
        },
        "protocol",
    )
    _require_exact_keys(
        expected_profile,
        {"id", "revision", "sha256"},
        "expected_profile",
    )

    transport_kind = _nonempty_string(
        transport["kind"],
        "transport.kind",
    )
    if transport_kind != "usb_cdc_serial":
        raise HX1ConfigError(
            "transport.kind must be 'usb_cdc_serial'"
        )

    client_minor = _nonnegative_int(
        protocol["client_minor"],
        "protocol.client_minor",
    )
    if client_minor != PROTOCOL_MINOR:
        raise HX1ConfigError(
            f"protocol.client_minor must be {PROTOCOL_MINOR}"
        )

    required_server_minor = _nonnegative_int(
        protocol["required_server_minor"],
        "protocol.required_server_minor",
    )
    if required_server_minor > client_minor:
        raise HX1ConfigError(
            "required_server_minor must not exceed client_minor"
        )

    joint_count = _positive_int(
        protocol["joint_count"],
        "protocol.joint_count",
    )
    if joint_count != 18:
        raise HX1ConfigError("protocol.joint_count must be 18")

    heartbeat_period_ms = _positive_int(
        protocol["heartbeat_period_ms"],
        "protocol.heartbeat_period_ms",
    )
    target_period_ms = _positive_int(
        protocol["target_period_ms"],
        "protocol.target_period_ms",
    )

    # Contract sanity. Keep normal Pi cadence well inside accepted MCU
    # watchdogs without making the Pi transport the safety authority.
    if heartbeat_period_ms >= 750:
        raise HX1ConfigError(
            "heartbeat period must be below the 750 ms link timeout"
        )
    if target_period_ms >= 200:
        raise HX1ConfigError(
            "target period must be below the 200 ms motion timeout"
        )

    return HX1ClientConfig(
        schema_version=1,
        link_id=_nonempty_string(
            data["link_id"],
            "link_id",
        ),
        transport_kind=transport_kind,
        device_path=_nonempty_string(
            transport["device"],
            "transport.device",
        ),
        baudrate=_positive_int(
            transport["baudrate"],
            "transport.baudrate",
        ),
        write_timeout_s=_positive_float(
            transport["write_timeout_s"],
            "transport.write_timeout_s",
        ),
        client_minor=client_minor,
        required_server_minor=required_server_minor,
        joint_count=joint_count,
        heartbeat_period_ms=heartbeat_period_ms,
        target_period_ms=target_period_ms,
        expected_profile_id=_nonempty_string(
            expected_profile["id"],
            "expected_profile.id",
        ),
        expected_profile_revision=_positive_int(
            expected_profile["revision"],
            "expected_profile.revision",
        ),
        expected_profile_sha256=_sha256_text(
            expected_profile["sha256"],
            "expected_profile.sha256",
        ),
    )


def load_hx1_client_config(
    path: str | Path,
) -> HX1ClientConfig:
    path = Path(path)
    try:
        text = path.read_text(encoding="utf-8")
    except OSError as exc:
        raise HX1ConfigError(
            f"could not read HX1 client config {path}: {exc}"
        ) from exc

    try:
        data = json.loads(text)
    except json.JSONDecodeError as exc:
        raise HX1ConfigError(
            f"invalid JSON in HX1 client config {path}: {exc}"
        ) from exc

    return parse_hx1_client_config(data)
