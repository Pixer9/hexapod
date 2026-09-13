"""Internal helpers for validating dynamically supplied numeric values."""

from __future__ import annotations

from collections.abc import Buffer
from typing import SupportsFloat, SupportsIndex


def float_from_unknown(value: object) -> float:
    """Convert a dynamically supplied value using Python's float semantics.

    Callers remain responsible for rejecting booleans and enforcing finite,
    sign, range, or other domain-specific constraints.
    """
    if isinstance(value, (str, Buffer, SupportsFloat, SupportsIndex)):
        return float(value)

    raise TypeError("value cannot be converted to float")
