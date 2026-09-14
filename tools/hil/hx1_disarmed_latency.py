#!/usr/bin/env python3
"""DISARMED-only HX1 STAGE latency qualification for the Servo 2040.

This tool never sends ARM, START, TARGET, STOP, DISARM, ESTOP, CLEAR_FAULT, or
CLEAR_ESTOP. It negotiates a session, verifies the MCU is DISARMED/NONE, then
streams valid STAGE commands at a fixed rate and measures ACK latency.

STAGE changes only the staged software target while DISARMED; it does not enable
PWM. The tool aborts if any observed STATUS leaves DISARMED/NONE.
"""

from __future__ import annotations

import argparse
import statistics
import sys
import time
from dataclasses import dataclass
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))

from hexapod.hx1 import (  # noqa: E402
    HX1Ack,
    HX1ClientCore,
    HX1Event,
    HX1Info,
    HX1Link,
    HX1Nack,
    HX1Status,
    create_serial_hx1_transport,
    load_hx1_client_config,
)
from tools.hil.stationary_runtime import (  # noqa: E402
    HX1_CONFIG,
    build_motion_pipeline,
)

DEFAULT_RATE_HZ = 50.0
DEFAULT_COMMANDS = 250
DEFAULT_DRAIN_S = 3.0
DEFAULT_MEDIAN_MAX_MS = 20.0
DEFAULT_MAX_MS = 40.0
DEFAULT_TREND_MAX_MS = 5.0


@dataclass(frozen=True, slots=True)
class Result:
    sent: int
    acked: int
    nacks: int
    missing: int
    duplicate_acks: int
    latencies_ms: tuple[float, ...]
    send_intervals_ms: tuple[float, ...]
    first_half_avg_ms: float
    second_half_avg_ms: float
    trend_delta_ms: float
    final_status: HX1Status


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Benchmark HX1 STAGE latency while the Servo 2040 remains DISARMED."
    )
    parser.add_argument("--rate-hz", type=float, default=DEFAULT_RATE_HZ)
    parser.add_argument("--commands", type=int, default=DEFAULT_COMMANDS)
    parser.add_argument("--drain-s", type=float, default=DEFAULT_DRAIN_S)
    parser.add_argument(
        "--median-max-ms",
        type=float,
        default=DEFAULT_MEDIAN_MAX_MS,
    )
    parser.add_argument("--max-ms", type=float, default=DEFAULT_MAX_MS)
    parser.add_argument("--trend-max-ms", type=float, default=DEFAULT_TREND_MAX_MS)
    return parser.parse_args()


def _percentile(values: list[float], percentile: float) -> float:
    """Linear-interpolated percentile matching the usual inclusive definition."""
    ordered = sorted(values)
    if not ordered:
        raise ValueError("percentile requires at least one value")
    if len(ordered) == 1:
        return ordered[0]

    position = (len(ordered) - 1) * percentile
    lower = int(position)
    upper = min(lower + 1, len(ordered) - 1)
    fraction = position - lower
    return ordered[lower] + (ordered[upper] - ordered[lower]) * fraction


def _require_safe_status(status: HX1Status, session: str) -> None:
    if status.session != session:
        raise RuntimeError(
            f"STATUS session mismatch: expected {session}, got {status.session}"
        )
    if status.state != "DISARMED" or status.fault != "NONE":
        raise RuntimeError(
            "Refusing latency benchmark unless MCU is DISARMED/NONE; "
            f"observed state={status.state} fault={status.fault}"
        )


def _poll_until_info(link: HX1Link, hello_seq: int, timeout_s: float) -> HX1Info:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        for message in link.poll():
            if isinstance(message, HX1Info) and message.ref_seq == hello_seq:
                return message
            if isinstance(message, HX1Nack) and message.ref_seq == hello_seq:
                raise RuntimeError(
                    f"HELLO NACK: command={message.command} error={message.error}"
                )
        time.sleep(0.001)
    raise TimeoutError("timed out waiting for HELLO/INFO")


def _poll_until_status(
    link: HX1Link,
    session: str,
    timeout_s: float,
) -> HX1Status:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        for message in link.poll():
            if isinstance(message, HX1Status) and message.session == session:
                return message
        time.sleep(0.001)
    raise TimeoutError("timed out waiting for STATUS")


def _consume_messages(
    link: HX1Link,
    session: str,
    sent_at: dict[int, float],
    acked_refs: set[int],
    latencies_ms: list[float],
) -> tuple[int, int]:
    nacks = 0
    duplicates = 0

    for message in link.poll():
        received_at = time.monotonic()

        if isinstance(message, HX1Ack) and message.command == "STAGE":
            sent = sent_at.get(message.ref_seq)
            if sent is None:
                continue
            if message.ref_seq in acked_refs:
                duplicates += 1
                continue
            acked_refs.add(message.ref_seq)
            latencies_ms.append((received_at - sent) * 1000.0)
            continue

        if isinstance(message, HX1Nack) and message.command == "STAGE":
            if message.ref_seq in sent_at:
                nacks += 1
                print(
                    "NACK:",
                    f"ref_seq={message.ref_seq}",
                    f"error={message.error}",
                )
            continue

        if isinstance(message, HX1Status):
            _require_safe_status(message, session)
            continue

        if isinstance(message, HX1Event):
            print("EVENT:", message.event_type, message.detail)

    return nacks, duplicates


def run_benchmark(rate_hz: float, commands: int, drain_s: float) -> Result:
    if not 0.0 < rate_hz <= 200.0:
        raise ValueError("--rate-hz must be > 0 and <= 200")
    if commands < 10:
        raise ValueError("--commands must be at least 10")
    if not 0.1 <= drain_s <= 10.0:
        raise ValueError("--drain-s must be between 0.1 and 10 seconds")

    config = load_hx1_client_config(HX1_CONFIG)
    client = HX1ClientCore(config)
    transport = create_serial_hx1_transport(config)
    link = HX1Link(client, transport)

    pipeline = build_motion_pipeline()
    initialization = pipeline.initialize_flat_stance()
    staged_vector = initialization.staged_joint_vector_deg

    sent_at: dict[int, float] = {}
    acked_refs: set[int] = set()
    latencies_ms: list[float] = []
    send_times: list[float] = []
    nacks = 0
    duplicates = 0

    link.open()
    try:
        hello = client.hello()
        link.send(hello)
        info = _poll_until_info(link, hello.seq, timeout_s=2.0)
        if client.session is None:
            raise RuntimeError("HELLO/INFO completed without negotiated session")

        get_status = client.get_status()
        link.send(get_status)
        initial_status = _poll_until_status(link, client.session, timeout_s=2.0)
        _require_safe_status(initial_status, client.session)

        print("HX1 DISARMED STAGE latency benchmark")
        print("Device:", config.device_path)
        print("Firmware:", info.firmware_version)
        print("Rate:", f"{rate_hz:.1f} Hz")
        print("Commands:", commands)
        print("No ARM / START / TARGET commands will be sent.")
        print(
            "Initial:",
            f"state={initial_status.state}",
            f"fault={initial_status.fault}",
            f"session={client.session}",
        )
        print()

        period_s = 1.0 / rate_hz
        next_send = time.monotonic()

        for _ in range(commands):
            while True:
                now = time.monotonic()
                if now >= next_send:
                    break
                new_nacks, new_duplicates = _consume_messages(
                    link,
                    client.session,
                    sent_at,
                    acked_refs,
                    latencies_ms,
                )
                nacks += new_nacks
                duplicates += new_duplicates
                remaining = next_send - time.monotonic()
                if remaining > 0.001:
                    time.sleep(min(remaining / 2.0, 0.001))

            outbound = client.stage_degrees(staged_vector)
            sent = time.monotonic()
            link.send(outbound)
            sent_at[outbound.seq] = sent
            send_times.append(sent)

            new_nacks, new_duplicates = _consume_messages(
                link,
                client.session,
                sent_at,
                acked_refs,
                latencies_ms,
            )
            nacks += new_nacks
            duplicates += new_duplicates

            next_send = sent + period_s

        drain_deadline = time.monotonic() + drain_s
        while len(acked_refs) + nacks < commands and time.monotonic() < drain_deadline:
            new_nacks, new_duplicates = _consume_messages(
                link,
                client.session,
                sent_at,
                acked_refs,
                latencies_ms,
            )
            nacks += new_nacks
            duplicates += new_duplicates
            time.sleep(0.001)

        get_status = client.get_status()
        link.send(get_status)
        final_status = _poll_until_status(link, client.session, timeout_s=2.0)
        _require_safe_status(final_status, client.session)

    finally:
        link.close()

    send_intervals_ms = [
        (later - earlier) * 1000.0 for earlier, later in zip(send_times, send_times[1:])
    ]

    # ACK latency samples are appended in receive order. ACKs are emitted in
    # command order by the single-threaded MCU, so splitting this sequence is a
    # direct backlog/trend check.
    midpoint = len(latencies_ms) // 2
    first = latencies_ms[:midpoint]
    second = latencies_ms[midpoint:]
    first_avg = statistics.fmean(first) if first else float("nan")
    second_avg = statistics.fmean(second) if second else float("nan")

    return Result(
        sent=commands,
        acked=len(acked_refs),
        nacks=nacks,
        missing=commands - len(acked_refs) - nacks,
        duplicate_acks=duplicates,
        latencies_ms=tuple(latencies_ms),
        send_intervals_ms=tuple(send_intervals_ms),
        first_half_avg_ms=first_avg,
        second_half_avg_ms=second_avg,
        trend_delta_ms=second_avg - first_avg,
        final_status=final_status,
    )


def print_result(result: Result) -> None:
    print("Results")
    print("-------")
    print(f"Sent:          {result.sent:4d}")
    print(f"ACK:           {result.acked:4d}")
    print(f"NACK:          {result.nacks:4d}")
    print(f"Missing:       {result.missing:4d}")
    print(f"Duplicate ACK: {result.duplicate_acks:4d}")
    print()

    if result.latencies_ms:
        values = list(result.latencies_ms)
        print("ACK latency")
        print("-----------")
        print(f"min:     {min(values):10.3f} ms")
        print(f"median:  {statistics.median(values):10.3f} ms")
        print(f"average: {statistics.fmean(values):10.3f} ms")
        print(f"P95:     {_percentile(values, 0.95):10.3f} ms")
        print(f"P99:     {_percentile(values, 0.99):10.3f} ms")
        print(f"max:     {max(values):10.3f} ms")
        print()
        print(f"first-half avg:  {result.first_half_avg_ms:10.3f} ms")
        print(f"second-half avg: {result.second_half_avg_ms:10.3f} ms")
        print(f"trend delta:     {result.trend_delta_ms:+10.3f} ms")
        print()

    if result.send_intervals_ms:
        intervals = result.send_intervals_ms
        print("Host send interval")
        print("------------------")
        print(f"average: {statistics.fmean(intervals):8.3f} ms")
        print(f"min:     {min(intervals):8.3f} ms")
        print(f"max:     {max(intervals):8.3f} ms")
        print()

    print(
        "Final MCU:",
        f"state={result.final_status.state}",
        f"fault={result.final_status.fault}",
    )


def passes(result: Result, args: argparse.Namespace) -> bool:
    if result.acked != result.sent:
        return False
    if result.nacks or result.missing or result.duplicate_acks:
        return False
    if not result.latencies_ms:
        return False
    if statistics.median(result.latencies_ms) > args.median_max_ms:
        return False
    if max(result.latencies_ms) > args.max_ms:
        return False
    if result.trend_delta_ms > args.trend_max_ms:
        return False
    if result.final_status.state != "DISARMED" or result.final_status.fault != "NONE":
        return False
    return True


def main() -> int:
    args = parse_args()
    result = run_benchmark(args.rate_hz, args.commands, args.drain_s)
    print_result(result)

    passed = passes(result, args)
    print()
    print("PASS" if passed else "FAIL")
    if not passed:
        print(
            "Acceptance:",
            f"all ACK, no errors, median <= {args.median_max_ms:.1f} ms,",
            f"max <= {args.max_ms:.1f} ms,",
            f"trend <= +{args.trend_max_ms:.1f} ms",
        )
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
