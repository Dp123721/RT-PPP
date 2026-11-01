"""Tests orbit and clock comparison metrics on synthetic products."""

from __future__ import annotations

import contextlib
import io
import json
import math
import tempfile
import unittest
from pathlib import Path

import numpy as np

from _support import RESEARCH_ROOT  # noqa: F401
from rtppp_analysis.cli import main as cli_main
from rtppp_analysis.models import ClockPoint, OrbitPoint, Product
from rtppp_analysis.orbit.compare import (
    SPEED_OF_LIGHT_MPS,
    OrbitInterpolator,
    _clock_at,
    compare_product,
    sisre_m,
)


def orbit_point(epoch_seconds: int, position: list[float]) -> OrbitPoint:
    return OrbitPoint(
        epoch_ns=epoch_seconds * 1_000_000_000,
        satellite="G01",
        position_m=np.asarray(position, dtype=float),
        velocity_mps=np.asarray([0.0, 2000.0, 0.0]),
    )


class OrbitInterpolationTests(unittest.TestCase):
    def test_interpolation_is_bounded_and_rejects_large_gaps(self) -> None:
        interpolator = OrbitInterpolator(
            [
                orbit_point(0, [7_000_000.0, -10_000.0, 0.0]),
                orbit_point(10, [7_000_000.0, 10_000.0, 0.0]),
            ]
        )
        position, velocity = interpolator.at(5_000_000_000, 2, 10.0)  # type: ignore[misc]
        np.testing.assert_allclose(position, [7_000_000.0, 0.0, 0.0], atol=1e-9)
        np.testing.assert_allclose(velocity, [0.0, 2000.0, 0.0], atol=1e-12)
        self.assertIsNone(interpolator.at(-1, 2, 10.0))
        self.assertIsNone(interpolator.at(11_000_000_000, 2, 10.0))
        self.assertIsNone(interpolator.at(5_000_000_000, 2, 9.9))

    def test_clock_interpolation_is_bounded_and_rejects_large_gaps(self) -> None:
        points = [ClockPoint(0, "G01", 1e-6), ClockPoint(10_000_000_000, "G01", 3e-6)]
        self.assertAlmostEqual(_clock_at(points, 5_000_000_000, 10.0), 2e-6)
        self.assertTrue(math.isnan(_clock_at(points, -1, 10.0)))
        self.assertTrue(math.isnan(_clock_at(points, 11_000_000_000, 10.0)))
        self.assertTrue(math.isnan(_clock_at(points, 5_000_000_000, 9.9)))

    def test_sisre_uses_datum_removed_clock_and_transverse_weight(self) -> None:
        self.assertAlmostEqual(sisre_m(1.0, 7.0, 0.0, 1.0), 1.0)
        self.assertAlmostEqual(sisre_m(0.0, 7.0, 0.0, 0.0), 1.0)
        self.assertTrue(math.isnan(sisre_m(0.0, 0.0, 0.0, float("nan"))))

    def test_compare_recovers_one_meter_radial_and_clock_datum(self) -> None:
        reference_orbits: dict[str, list[OrbitPoint]] = {}
        candidate_orbits: dict[str, list[OrbitPoint]] = {}
        reference_clocks: dict[str, list[ClockPoint]] = {}
        candidate_clocks: dict[str, list[ClockPoint]] = {}
        for satellite, clock_offset in (("G01", 10e-9), ("G02", 30e-9)):
            reference_orbits[satellite] = [
                OrbitPoint(
                    0,
                    satellite,
                    np.asarray([7_000_000.0, -10_000.0, 0.0]),
                    velocity_mps=np.asarray([0.0, 2000.0, 0.0]),
                ),
                OrbitPoint(
                    10_000_000_000,
                    satellite,
                    np.asarray([7_000_000.0, 10_000.0, 0.0]),
                    velocity_mps=np.asarray([0.0, 2000.0, 0.0]),
                ),
            ]
            candidate_orbits[satellite] = [
                OrbitPoint(
                    5_000_000_000,
                    satellite,
                    np.asarray([7_000_001.0, 0.0, 0.0]),
                )
            ]
            reference_clocks[satellite] = [
                ClockPoint(0, satellite, 0.0),
                ClockPoint(10_000_000_000, satellite, 0.0),
            ]
            candidate_clocks[satellite] = [
                ClockPoint(5_000_000_000, satellite, clock_offset)
            ]

        reference = Product(
            "reference", "UTC", "COM", "IGS20", reference_orbits, reference_clocks, []
        )
        candidate = Product(
            "candidate", "UTC", "COM", "IGS20", candidate_orbits, candidate_clocks, []
        )
        rows, coverage = compare_product(
            candidate,
            reference,
            {
                "orbit_interpolation_points": 2,
                "max_orbit_gap_seconds": 10.0,
                "max_clock_gap_seconds": 10.0,
            },
            "reference",
        )
        rows.sort(key=lambda row: row["satellite"])
        self.assertEqual(len(rows), 2)
        for row in rows:
            self.assertAlmostEqual(row["dx_m"], 1.0, places=9)
            self.assertAlmostEqual(row["dy_m"], 0.0, places=9)
            self.assertAlmostEqual(row["radial_m"], 1.0, places=9)
            self.assertAlmostEqual(row["along_m"], 0.0, places=9)
            self.assertAlmostEqual(row["cross_m"], 0.0, places=9)
            self.assertAlmostEqual(row["orbit_3d_m"], 1.0, places=9)
        self.assertAlmostEqual(rows[0]["clock_raw_m"], SPEED_OF_LIGHT_MPS * 10e-9)
        self.assertAlmostEqual(rows[1]["clock_raw_m"], SPEED_OF_LIGHT_MPS * 30e-9)
        self.assertAlmostEqual(rows[0]["clock_datum_removed_m"], -SPEED_OF_LIGHT_MPS * 10e-9)
        self.assertAlmostEqual(rows[1]["clock_datum_removed_m"], SPEED_OF_LIGHT_MPS * 10e-9)
        self.assertAlmostEqual(rows[0]["sisre_m"], 1.0 + SPEED_OF_LIGHT_MPS * 10e-9)
        self.assertAlmostEqual(rows[1]["sisre_m"], abs(1.0 - SPEED_OF_LIGHT_MPS * 10e-9))
        self.assertEqual(coverage["candidate_sample_count"], 2)
        self.assertEqual(coverage["compared_orbit_count"], 2)
        self.assertEqual(coverage["clock_compared_count"], 2)
        self.assertEqual(coverage["rejection_counts"], {})

    def test_compare_reports_orbit_gap_rejection_and_zero_coverage(self) -> None:
        reference = Product(
            "reference",
            "UTC",
            "COM",
            "IGS20",
            {
                "G01": [
                    orbit_point(0, [7_000_000.0, -20_000.0, 0.0]),
                    orbit_point(20, [7_000_000.0, 20_000.0, 0.0]),
                ]
            },
            {},
            [],
        )
        candidate = Product(
            "candidate",
            "UTC",
            "COM",
            "IGS20",
            {"G01": [orbit_point(10, [7_000_001.0, 0.0, 0.0])]},
            {},
            [],
        )
        rows, coverage = compare_product(
            candidate,
            reference,
            {
                "orbit_interpolation_points": 2,
                "max_orbit_gap_seconds": 10.0,
                "max_clock_gap_seconds": 10.0,
            },
            "reference",
        )
        self.assertEqual(rows, [])
        self.assertEqual(coverage["compared_orbit_count"], 0)
        self.assertEqual(coverage["orbit_coverage_rate"], 0.0)
        self.assertEqual(coverage["rejection_counts"], {"orbit_gap_exceeded": 1})


class OrbitCliTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.directory = Path(self.temporary.name)

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def write_sp3(self, name: str, radial_offset_m: float) -> None:
        lines = ["#dP synthetic orbit", "## synthetic time"]
        for second in (0, 10, 20):
            lines.append(f"*  2026 07 27 00 00 {second:02d}.00000000")
            for satellite, base_x_km in (("G01", 7000.0), ("G02", 7100.0)):
                x_km = base_x_km + radial_offset_m / 1000.0
                y_km = -1.0 + second / 10.0
                lines.append(f"P{satellite} {x_km:.6f} {y_km:.6f} 0.000000 0.000000")
                lines.append(f"V{satellite} 0.000000 1000.000000 0.000000 0.000000")
        lines.append("EOF")
        (self.directory / name).write_text("\n".join(lines) + "\n", encoding="utf-8")

    def write_clk(self, name: str, first_offset_s: float, second_offset_s: float) -> None:
        lines = ["     3.04           C                   RINEX VERSION / TYPE", "END OF HEADER"]
        for second in (0, 10, 20):
            lines.append(
                f"AS G01 2026 07 27 00 00 {second:02d}.000000 1 {first_offset_s:.12e}"
            )
            lines.append(
                f"AS G02 2026 07 27 00 00 {second:02d}.000000 1 {second_offset_s:.12e}"
            )
        (self.directory / name).write_text("\n".join(lines) + "\n", encoding="utf-8")

    def test_orbit_cli_writes_valid_complete_outputs(self) -> None:
        self.write_sp3("reference.sp3", 0.0)
        self.write_sp3("center-a.sp3", 1.0)
        self.write_sp3("center-b.sp3", 2.0)
        self.write_clk("reference.clk", 0.0, 0.0)
        self.write_clk("center-a.clk", 10e-9, 30e-9)
        self.write_clk("center-b.clk", 20e-9, 40e-9)
        config = {
            "analysis": "orbit_clock",
            "output_directory": "results",
            "reference": {
                "id": "reference",
                "sp3": "reference.sp3",
                "clk": "reference.clk",
                "time_scale": "UTC",
                "orbit_reference": "COM",
                "reference_frame": "IGS20",
            },
            "candidates": [
                {
                    "id": "center-a",
                    "sp3": "center-a.sp3",
                    "clk": "center-a.clk",
                    "time_scale": "UTC",
                    "orbit_reference": "COM",
                    "reference_frame": "IGS20",
                },
                {
                    "id": "center-b",
                    "sp3": "center-b.sp3",
                    "clk": "center-b.clk",
                    "time_scale": "UTC",
                    "orbit_reference": "COM",
                    "reference_frame": "IGS20",
                },
            ],
            "options": {
                "orbit_interpolation_points": 3,
                "max_orbit_gap_seconds": 10.0,
                "max_clock_gap_seconds": 10.0,
                "clock_datum": "constellation_epoch_median",
                "pairwise_centers": True,
            },
        }
        config_path = self.directory / "orbit.json"
        config_path.write_text(json.dumps(config), encoding="utf-8")
        stdout = io.StringIO()
        stderr = io.StringIO()
        with contextlib.redirect_stdout(stdout), contextlib.redirect_stderr(stderr):
            exit_code = cli_main(["orbit", "--config", str(config_path)])
        self.assertEqual(exit_code, 0, stderr.getvalue())

        output = self.directory / "results"
        for name in (
            "orbit_clock_residuals.csv",
            "orbit_clock_summary.csv",
            "orbit_clock_satellite_summary.csv",
            "metrics.json",
            "orbit_clock_histograms.png",
            "orbit_clock_satellite_rms.png",
            "orbit_clock_timeseries.png",
            "report.html",
            "manifest.json",
        ):
            with self.subTest(name=name):
                self.assertTrue((output / name).is_file(), name)
        metrics = json.loads((output / "metrics.json").read_text(encoding="utf-8"))
        self.assertEqual(metrics["row_count"], 18)
        self.assertEqual(metrics["common_sample_count"], 6)
        self.assertEqual({item["scope"] for item in metrics["metrics"]}, {"all", "common", "pairwise"})
        self.assertEqual(len(metrics["coverage"]), 3)
        self.assertEqual(len(metrics["satellite_metrics"]), 6)
        self.assertTrue(
            all(item["candidate_sample_count"] == 3 for item in metrics["satellite_metrics"])
        )
        self.assertEqual(metrics["warnings"], [])
        products = {item["id"]: item for item in metrics["products"]}
        self.assertEqual(products["reference"]["time_scale"], "UTC")
        self.assertEqual(products["center-a"]["orbit_reference"], "COM")
        self.assertEqual(products["center-b"]["reference_frame"], "IGS20")
        manifest = json.loads((output / "manifest.json").read_text(encoding="utf-8"))
        self.assertEqual(manifest["analysis"], "orbit_clock")
        self.assertEqual(len(manifest["inputs"]), 6)
        self.assertTrue(all(item["sha256"] for item in manifest["inputs"]))


if __name__ == "__main__":
    unittest.main()
