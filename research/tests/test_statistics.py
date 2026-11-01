"""Tests scientific statistics and robust event metrics."""

from __future__ import annotations

import math
import unittest

import numpy as np

from _support import RESEARCH_ROOT  # noqa: F401
from rtppp_analysis.core.statistics import describe, sustained_convergence_seconds
from rtppp_analysis.io.tables import json_value


class StatisticsTests(unittest.TestCase):
    def test_describe_ignores_non_finite_and_reports_coverage(self) -> None:
        result = describe([1.0, 2.0, np.nan, np.inf])
        self.assertEqual(result["count"], 2)
        self.assertEqual(result["total_count"], 4)
        self.assertEqual(result["missing_count"], 2)
        self.assertEqual(result["coverage_rate"], 0.5)
        self.assertAlmostEqual(result["rms"], math.sqrt(2.5))

    def test_empty_statistics_serialize_as_null(self) -> None:
        result = describe([])
        self.assertEqual(result["count"], 0)
        self.assertEqual(result["total_count"], 0)
        normalized = json_value(result)
        self.assertIsNone(normalized["coverage_rate"])
        self.assertIsNone(normalized["rms"])

    def test_sustained_convergence(self) -> None:
        epochs = np.arange(5, dtype=np.int64) * 1_000_000_000
        horizontal = np.array([0.2, 0.09, 0.08, 0.07, 0.06])
        vertical = np.array([0.3, 0.19, 0.18, 0.17, 0.16])
        result = sustained_convergence_seconds(epochs, horizontal, vertical, 0.1, 0.2, 2.0, 1.5)
        self.assertEqual(result, 1.0)

    def test_convergence_does_not_cross_large_gap(self) -> None:
        epochs = np.array([0, 1, 10, 11], dtype=np.int64) * 1_000_000_000
        values = np.zeros(4)
        result = sustained_convergence_seconds(epochs, values, values, 0.1, 0.1, 2.0, 1.5)
        self.assertTrue(math.isnan(result))

    def test_non_increasing_convergence_epochs_are_rejected(self) -> None:
        epochs = np.array([0, 1, 1], dtype=np.int64)
        values = np.zeros(3)
        with self.assertRaises(ValueError):
            sustained_convergence_seconds(epochs, values, values, 0.1, 0.1, 1.0)


if __name__ == "__main__":
    unittest.main()
