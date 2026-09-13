"""Tests for shared dynamic numeric conversion helpers."""

from __future__ import annotations

import unittest

from hexapod._numeric import float_from_unknown


class FloatFromUnknownTests(unittest.TestCase):
    def test_accepts_common_float_convertible_values(self) -> None:
        self.assertEqual(float_from_unknown(12), 12.0)
        self.assertEqual(float_from_unknown(12.5), 12.5)
        self.assertEqual(float_from_unknown("12.5"), 12.5)
        self.assertEqual(float_from_unknown(b"12.5"), 12.5)

    def test_accepts_supports_float(self) -> None:
        class FloatValue:
            def __float__(self) -> float:
                return 3.25

        self.assertEqual(float_from_unknown(FloatValue()), 3.25)

    def test_accepts_supports_index(self) -> None:
        class IndexValue:
            def __index__(self) -> int:
                return 7

        self.assertEqual(float_from_unknown(IndexValue()), 7.0)

    def test_rejects_non_convertible_object(self) -> None:
        with self.assertRaises(TypeError):
            float_from_unknown(object())

    def test_boolean_policy_belongs_to_caller(self) -> None:
        self.assertEqual(float_from_unknown(True), 1.0)


if __name__ == "__main__":
    unittest.main()
