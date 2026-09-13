#!/usr/bin/env python3
"""Read-only hardware-in-the-loop probe for the HX1 Servo 2040 link.

The probe performs only HELLO/INFO negotiation and GET_STATUS. It does not
stage, arm, start, target, stop, disarm, clear faults, or otherwise command
actuator motion.
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path

from hexapod.hx1 import (
    HX1ClientCore,
    HX1Event,
    HX1Info,
    HX1Link,
    HX1Nack,
    HX1Status,
    create_serial_hx1_transport,
    load_hx1_client_config,
)

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG_PATH = REPO_ROOT / "config" / "hardware" / "servo2040.json"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Perform a read-only HX1 HIL probe against the configured Servo 2040."
        )
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=DEFAULT_CONFIG_PATH,
        help=(f"HX1 client configuration file (default: {DEFAULT_CONFIG_PATH})"),
    )
    return parser.parse_args()


def describe(message: object) -> str:
    if isinstance(message, HX1Info):
        return (
            "INFO "
            f"session={message.session} "
            f"server_minor={message.server_minor} "
            f"fw={message.firmware_version} "
            f"mcu={message.mcu_id} "
            f"profile={message.profile_id} "
            f"rev={message.profile_revision} "
            f"joints={message.joint_count} "
            f"state={message.state}"
        )

    if isinstance(message, HX1Status):
        return (
            "STATUS "
            f"session={message.session} "
            f"state={message.state} "
            f"fault={message.fault} "
            f"last_target_seq={message.last_target_seq} "
            f"target_age_ms={message.target_age_ms} "
            f"heartbeat_age_ms={message.heartbeat_age_ms} "
            f"command_valid={message.command_valid}"
        )

    if isinstance(message, HX1Event):
        return f"EVENT type={message.event_type} detail={message.detail}"

    if isinstance(message, HX1Nack):
        return (
            "NACK "
            f"ref_seq={message.ref_seq} "
            f"command={message.command} "
            f"error={message.error}"
        )

    return repr(message)


def wait_for(
    link: HX1Link,
    wanted_type: type[HX1Info] | type[HX1Status],
    timeout_s: float,
) -> HX1Info | HX1Status:
    deadline = time.monotonic() + timeout_s

    while time.monotonic() < deadline:
        messages = link.poll()

        for message in messages:
            print("RX:", describe(message))

            if isinstance(message, wanted_type):
                return message

        time.sleep(0.01)

    raise TimeoutError(f"timed out waiting for {wanted_type.__name__}")


def main() -> int:
    args = parse_args()

    config = load_hx1_client_config(args.config)

    client = HX1ClientCore(config)
    transport = create_serial_hx1_transport(config)
    link = HX1Link(client, transport)

    print("Config:", args.config)
    print("Device:", config.device_path)

    try:
        print("OPEN")
        link.open()

        hello = client.hello()
        print(
            "TX: HELLO",
            f"seq={hello.seq}",
            f"profile={config.expected_profile_id}",
        )
        link.send(hello)

        info = wait_for(
            link,
            HX1Info,
            timeout_s=2.0,
        )

        if not isinstance(info, HX1Info):
            raise RuntimeError("unexpected response type")

        if info.ref_seq != hello.seq:
            raise RuntimeError("INFO did not reference the HELLO sequence")

        print("HELLO/INFO negotiation verified.")
        print("Negotiated session:", client.session)

        get_status = client.get_status()
        print(
            "TX: GET_STATUS",
            f"seq={get_status.seq}",
            f"session={client.session}",
        )
        link.send(get_status)

        status = wait_for(
            link,
            HX1Status,
            timeout_s=2.0,
        )

        if not isinstance(status, HX1Status):
            raise RuntimeError("unexpected response type")

        if status.session != client.session:
            raise RuntimeError("STATUS session does not match negotiated session")

        print()
        print("HX1 HIL probe successful.")
        print("MCU state:", status.state)
        print("MCU fault:", status.fault)

        return 0

    finally:
        print("CLOSE")
        link.close()


if __name__ == "__main__":
    raise SystemExit(main())
