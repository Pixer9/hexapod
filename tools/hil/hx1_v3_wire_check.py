#!/usr/bin/env python3
"""Host-only cross-boundary verification for the HX1 v3 command wire."""

from __future__ import annotations

import importlib.util
import sys
import tempfile
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
HOST_PROTOCOL = REPO_ROOT / "src" / "hexapod" / "hx1" / "protocol.py"
MCU_PROTOCOL = (
    REPO_ROOT / "firmware" / "servo2040" / "src" / "hexapod_mcu" / "protocol.py"
)
MCU_CONSTANTS = MCU_PROTOCOL.with_name("constants.py")


def _load_file(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"unable to load {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def main() -> int:
    host = _load_file("hx1_v3_host_protocol_check", HOST_PROTOCOL)

    with tempfile.TemporaryDirectory() as td:
        pkg = Path(td) / "hexapod_mcu"
        pkg.mkdir()
        (pkg / "__init__.py").write_text("", encoding="utf-8")
        (pkg / "constants.py").write_bytes(MCU_CONSTANTS.read_bytes())
        (pkg / "protocol.py").write_bytes(MCU_PROTOCOL.read_bytes())
        sys.path.insert(0, td)
        import hexapod_mcu.protocol as mcu

        session = "A1B2C3D4"
        joints = (
            -12000,
            -10500,
            -9000,
            -7500,
            -6000,
            -4500,
            -3200,
            -1500,
            0,
            1500,
            3200,
            4500,
            6000,
            7500,
            9000,
            10500,
            12000,
            13500,
        )
        vectors = (
            (0, "HELLO", (1, "hexapod-standard-v1")),
            (1, "HEARTBEAT", (session, 123456)),
            (2, "STAGE", (session, *joints)),
            (3, "ARM", (session,)),
            (4, "START", (session,)),
            (5, "TARGET", (session, 20, *joints)),
            (6, "STOP", (session,)),
            (7, "DISARM", (session,)),
            (8, "ESTOP", ("00000000", "wire_check")),
            (9, "CLEAR_ESTOP", (session,)),
            (10, "CLEAR_FAULT", (session,)),
            (11, "GET_STATUS", (session,)),
        )

        for seq, command, fields in vectors:
            host_frame = host.encode_frame(seq, command, *fields)
            mcu_frame = mcu.encode_frame(seq, command, *fields)
            if host_frame != mcu_frame:
                raise AssertionError(f"encoder mismatch for {command}")

            host_parsed = host.parse_frame(host_frame)
            mcu_parsed = mcu.parse_frame(mcu_frame)
            host_tuple = (
                host_parsed.seq,
                host_parsed.message_type,
                host_parsed.fields,
            )
            if host_tuple != mcu_parsed:
                raise AssertionError(f"parser mismatch for {command}")

        stage = host.encode_frame(2, "STAGE", session, *joints)
        target = host.encode_frame(5, "TARGET", session, 20, *joints)
        if len(stage) != 52 or len(target) != 56:
            raise AssertionError("unexpected realtime command frame size")

        _, _, stage_fields = mcu.parse_command_frame(stage)
        if stage_fields[1:] != joints:
            raise AssertionError("native STAGE joint decode mismatch")
        if not all(isinstance(value, int) for value in stage_fields[1:]):
            raise AssertionError("STAGE joints were not decoded as native ints")

        _, _, target_fields = mcu.parse_command_frame(target)
        if target_fields[1] != 20 or target_fields[2:] != joints:
            raise AssertionError("native TARGET decode mismatch")

        framer = mcu.LineFramer()
        if framer.feed(stage[:7]):
            raise AssertionError("fragmented STAGE completed too early")
        if framer.feed(stage[7:31]):
            raise AssertionError("fragmented STAGE completed too early")
        if framer.feed(stage[31:]) != [stage]:
            raise AssertionError("fragmented STAGE reconstruction failed")

        framer = mcu.LineFramer()
        if framer.feed(stage + target) != [stage, target]:
            raise AssertionError("back-to-back framing failed")

        damaged = bytearray(target)
        damaged[-1] ^= 1
        try:
            mcu.parse_command_frame(damaged)
        except mcu.CRCError:
            pass
        else:
            raise AssertionError("corrupted TARGET CRC was accepted")

        ack = host.encode_frame(100, "ACK", 2, "STAGE")
        if not ack.startswith(b"HX1|") or not ack.endswith(b"\n"):
            raise AssertionError("ASCII telemetry representation changed")
        if mcu.LineFramer().feed(ack) != [ack]:
            raise AssertionError("MCU ASCII diagnostic framing regressed")

    print("PASS: all Pi command encoders match the MCU implementation")
    print("PASS: STAGE=52 bytes and TARGET=56 bytes")
    print("PASS: MCU realtime decoder returns native integer joint vectors")
    print("PASS: fragmented and back-to-back binary frames reconstruct")
    print("PASS: command CRC corruption is rejected")
    print("PASS: ASCII MCU telemetry representation is preserved")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
