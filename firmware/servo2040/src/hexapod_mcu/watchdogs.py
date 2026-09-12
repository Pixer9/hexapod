"""Pure watchdog timing helpers for the Servo 2040 runtime.

The helpers use MicroPython's wrap-safe ``time.ticks_diff`` when available.
Under CPython they fall back to ordinary integer subtraction, which keeps the
same host-side tests deterministic.
"""

try:
    import time
except ImportError:
    time = None


def ticks_diff(now_ms, then_ms):
    """Return wrap-safe elapsed milliseconds when MicroPython provides it."""
    if time is not None and hasattr(time, "ticks_diff"):
        return time.ticks_diff(now_ms, then_ms)
    return now_ms - then_ms


def age_ms(now_ms, timestamp_ms):
    """Return non-negative age, or -1 when no timestamp exists."""
    if timestamp_ms is None:
        return -1

    age = ticks_diff(now_ms, timestamp_ms)
    if age < 0:
        # A negative age means the supplied timestamp is logically in the
        # future. Treat it as invalid rather than manufacturing freshness.
        return -1

    return age


def is_fresh(now_ms, timestamp_ms, max_age_ms):
    """Return True when timestamp exists and its age is <= max_age_ms."""
    age = age_ms(now_ms, timestamp_ms)
    return age >= 0 and age <= max_age_ms


def is_timed_out(now_ms, timestamp_ms, timeout_ms):
    """Return True when timestamp exists and its age is >= timeout_ms."""
    age = age_ms(now_ms, timestamp_ms)
    return age >= timeout_ms if age >= 0 else False
