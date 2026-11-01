"""Tests GNSS time conversion and local coordinate transforms."""

from __future__ import annotations

import math
import unittest
from datetime import datetime, timezone

import numpy as np

from _support import RESEARCH_ROOT  # noqa: F401
from rtppp_analysis.core.coordinates import (
    WGS84_A,
    WGS84_E2,
    covariance_ecef_to_enu,
    ecef_delta_to_enu,
    ecef_delta_to_rac,
    ecef_to_geodetic,
    rac_basis,
)
from rtppp_analysis.core.time import (
    datetime_to_unix_ns,
    gps_utc_offset,
    gps_week_sow_to_utc,
    unix_ns_to_datetime,
    utc_to_gps_week_sow,
)


UTC = timezone.utc


def geodetic_to_ecef(latitude: float, longitude: float, height: float) -> np.ndarray:
    sin_lat = math.sin(latitude)
    cos_lat = math.cos(latitude)
    prime_vertical = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    return np.array(
        [
            (prime_vertical + height) * cos_lat * math.cos(longitude),
            (prime_vertical + height) * cos_lat * math.sin(longitude),
            (prime_vertical * (1.0 - WGS84_E2) + height) * sin_lat,
        ]
    )


class TimeTests(unittest.TestCase):
    def test_gps_utc_roundtrip_across_2017_leap_boundary(self) -> None:
        values = (
            datetime(2016, 12, 31, 23, 59, 59, 123456, tzinfo=UTC),
            datetime(2017, 1, 1, 0, 0, 0, 123456, tzinfo=UTC),
        )
        self.assertEqual(gps_utc_offset(values[0]), 17)
        self.assertEqual(gps_utc_offset(values[1]), 18)
        for value in values:
            week, sow = utc_to_gps_week_sow(value)
            self.assertAlmostEqual((gps_week_sow_to_utc(week, sow) - value).total_seconds(), 0.0, places=6)

    def test_gps_week_boundary(self) -> None:
        value = datetime(2026, 7, 25, 23, 59, 59, 999999, tzinfo=UTC)
        week, sow = utc_to_gps_week_sow(value)
        restored = gps_week_sow_to_utc(week, sow)
        self.assertAlmostEqual((restored - value).total_seconds(), 0.0, places=6)

    def test_unix_nanoseconds_negative_and_microsecond_roundtrip(self) -> None:
        value = datetime(1969, 12, 31, 23, 59, 59, 123456, tzinfo=UTC)
        nanoseconds = datetime_to_unix_ns(value)
        self.assertLess(nanoseconds, 0)
        self.assertEqual(unix_ns_to_datetime(nanoseconds), value)

    def test_naive_datetime_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            utc_to_gps_week_sow(datetime(2026, 1, 1))


class CoordinateTests(unittest.TestCase):
    def test_equator_and_poles(self) -> None:
        latitude, longitude, height = ecef_to_geodetic(np.array([WGS84_A, 0.0, 0.0]))
        np.testing.assert_allclose([latitude, longitude, height], [0.0, 0.0, 0.0], atol=1e-8)
        semi_minor = WGS84_A * math.sqrt(1.0 - WGS84_E2)
        for sign in (-1.0, 1.0):
            latitude, _, height = ecef_to_geodetic(
                np.array([0.0, 0.0, sign * (semi_minor + 100.0)])
            )
            self.assertAlmostEqual(latitude, sign * math.pi / 2.0, places=13)
            self.assertAlmostEqual(height, 100.0, places=6)

    def test_general_geodetic_roundtrip(self) -> None:
        expected = (math.radians(39.9), math.radians(116.4), 53.2)
        ecef = geodetic_to_ecef(*expected)
        actual = ecef_to_geodetic(ecef)
        np.testing.assert_allclose(actual, expected, atol=1e-8)

    def test_enu_axes_and_explicit_neu_mapping(self) -> None:
        reference = np.array([WGS84_A, 0.0, 0.0])
        enu = ecef_delta_to_enu(np.array([1.0, 2.0, 3.0]), reference)
        np.testing.assert_allclose(enu, [2.0, 3.0, 1.0])
        neu = np.array([enu[1], enu[0], enu[2]])
        np.testing.assert_allclose(neu, [3.0, 2.0, 1.0])

    def test_covariance_rotation_and_component_order(self) -> None:
        reference = np.array([WGS84_A, 0.0, 0.0])
        covariance = np.diag([1.0, 4.0, 9.0])
        rotated = covariance_ecef_to_enu(covariance, reference)
        np.testing.assert_allclose(np.diag(rotated), [4.0, 9.0, 1.0])

    def test_rac_basis_and_radial_recovery(self) -> None:
        position = np.array([2.0, 0.0, 0.0])
        velocity = np.array([0.0, 3.0, 0.0])
        basis = rac_basis(position, velocity)
        np.testing.assert_allclose(basis @ basis.T, np.eye(3), atol=1e-14)
        self.assertGreater(np.linalg.det(basis), 0.0)
        np.testing.assert_allclose(
            ecef_delta_to_rac(np.array([1.0, 0.0, 0.0]), position, velocity),
            [1.0, 0.0, 0.0],
        )

    def test_collinear_rac_state_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            rac_basis(np.array([1.0, 0.0, 0.0]), np.array([2.0, 0.0, 0.0]))


if __name__ == "__main__":
    unittest.main()
