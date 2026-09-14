# HX1 binary-command feasibility microbenchmark.
# Safe: no hardware imports, no runtime lifecycle commands, no USB/HX1 traffic.

import gc
import struct
import time

from hexapod_mcu.protocol import crc16_ccitt_false
from hexapod_mcu.runtime import _parse_joint_vector

JOINTS = (
    0,
    1500,
    -1500,
    3200,
    -3200,
    4500,
    -4500,
    6000,
    -6000,
    7500,
    -7500,
    9000,
    -9000,
    10500,
    -10500,
    12000,
    -12000,
    13500,
)

ASCII_FIELDS = tuple(str(v) for v in JOINTS)
BINARY_JOINTS = struct.pack("<18h", *JOINTS)

# Representative proposed binary TARGET:
# magic(2), version(1), type(1), seq(4), session(4), period(2),
# joints(36), crc(2) = 52 bytes.
HEADER = struct.pack(
    "<2sBBIIH",
    b"HX",
    2,  # binary command wire version
    4,  # representative TARGET type
    123456789,  # sequence
    0xA1B2C3D4,  # session
    20,  # period ms
)
BODY = HEADER + BINARY_JOINTS
FRAME = BODY + struct.pack("<H", crc16_ccitt_false(BODY))

_ticks_us = getattr(time, "ticks_us", None)

if _ticks_us is not None:

    def tick():
        return _ticks_us()

    def diff_us(start):
        return time.ticks_diff(_ticks_us(), start)
else:

    def tick():
        return time.ticks_ms()

    def diff_us(start):
        return time.ticks_diff(time.ticks_ms(), start) * 1000


def bench(name, iterations, fn):
    gc.collect()
    start = tick()
    for _ in range(iterations):
        fn()
    elapsed = diff_us(start)
    print(
        "%-34s %8.3f ms/call  (%d calls)"
        % (name, elapsed / iterations / 1000.0, iterations)
    )


def ascii_parse():
    values = _parse_joint_vector(ASCII_FIELDS)
    if values != JOINTS:
        raise RuntimeError("ASCII result mismatch")


def binary_unpack():
    values = struct.unpack("<18h", BINARY_JOINTS)
    if values != JOINTS:
        raise RuntimeError("binary result mismatch")


def binary_unpack_from():
    values = struct.unpack_from("<18h", FRAME, 14)
    if values != JOINTS:
        raise RuntimeError("binary unpack_from result mismatch")


def binary_header_unpack():
    magic, version, msg_type, seq, session, period = struct.unpack_from(
        "<2sBBIIH", FRAME, 0
    )
    if (
        magic != b"HX"
        or version != 2
        or msg_type != 4
        or seq != 123456789
        or session != 0xA1B2C3D4
        or period != 20
    ):
        raise RuntimeError("header mismatch")


def binary_crc():
    crc16_ccitt_false(FRAME[:-2])


def full_binary_decode():
    magic, version, msg_type, seq, session, period = struct.unpack_from(
        "<2sBBIIH", FRAME, 0
    )
    joints = struct.unpack_from("<18h", FRAME, 14)
    received_crc = struct.unpack_from("<H", FRAME, 50)[0]
    expected_crc = crc16_ccitt_false(FRAME[:50])

    if (
        magic != b"HX"
        or version != 2
        or msg_type != 4
        or seq != 123456789
        or session != 0xA1B2C3D4
        or period != 20
        or joints != JOINTS
        or received_crc != expected_crc
    ):
        raise RuntimeError("full binary decode mismatch")


print("HX1 binary command feasibility profiler")
print("---------------------------------------")
print("ASCII joint characters:", sum(len(x) for x in ASCII_FIELDS))
print("Binary joint bytes:", len(BINARY_JOINTS))
print("Representative binary TARGET bytes:", len(FRAME))
print()

bench("ASCII _parse_joint_vector(18)", 50, ascii_parse)
bench("struct.unpack(<18h)", 200, binary_unpack)
bench("struct.unpack_from(<18h)", 200, binary_unpack_from)
bench("binary header unpack_from", 200, binary_header_unpack)
bench("CRC(binary 50-byte body)", 100, binary_crc)
bench("FULL binary TARGET decode+CRC", 100, full_binary_decode)

print()
print("No hardware or runtime commands were used.")
