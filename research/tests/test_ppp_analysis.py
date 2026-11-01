"""Tests PPP compact-file analysis and dataset comparisons."""

from __future__ import annotations

import contextlib
import csv
import io
import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from _support import compact_root, compact_solution
from rtppp_analysis.cli import main as cli_main
from rtppp_analysis.ppp.analyze import _validate_ar_epoch
from rtppp_analysis.ppp.reference import load_reference
from rtppp_analysis.ppp.residuals import ResidualError, read_residuals
from rtppp_analysis.ppp.statistics import (
    bootstrap_mean_ci,
    detect_robust_jumps,
    fixed_segments,
    loss_reconvergence_events,
    paired_sign_test_pvalue,
)


class PppAnalysisTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.directory = Path(self.temporary.name)

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def write_dataset(self, name: str, offset: float) -> Path:
        reference = np.array([6378137.0, 0.0, 0.0])
        solutions = []
        for index in range(6):
            north = 0.02 + offset + index * 0.001
            east = -0.01 + offset
            up = 0.03
            fixed = 0 if index < 2 else (2 if index < 4 else 4)
            status = 1 if fixed == 0 else (2 if fixed < 4 else 3)
            solution = compact_solution(index * 1000)
            solution[1] = 2
            solution[2] = status
            solution[3] = [reference[0] + up, east, north]
            solution[4] = [north, east, up]
            solution[5] = [0.01, 0.0, 0.0, 0.01, 0.0, 0.01]
            solution[7] = [8, 4, fixed, 1.2]
            solution[8] = [list(range(1, 9)), list(range(1, 5)), list(range(1, fixed + 1))]
            solutions.append(solution)
        path = self.directory / name
        path.write_text(json.dumps(compact_root(solutions)), encoding="utf-8")
        return path

    def write_residuals(self) -> Path:
        directory = self.directory / "telemetry"
        directory.mkdir()
        path = directory / "ppp_residual_20260727.csv"
        fields = [
            "source_id", "epoch_utc", "satellite", "signal", "observation_type",
            "prefit_m", "postfit_m", "sigma_m", "elevation_deg", "adopted", "reason",
        ]
        with path.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            writer.writerows([
                {"source_id": "TEST", "epoch_utc": "2026-07-27T00:00:00.000Z",
                 "satellite": "G01", "signal": "c1", "observation_type": "code",
                 "prefit_m": 1.0, "postfit_m": 0.01, "sigma_m": 1.0,
                 "elevation_deg": 30.0, "adopted": "true", "reason": ""},
                {"source_id": "TEST", "epoch_utc": "2026-07-27T00:00:01.000Z",
                 "satellite": "G01", "signal": "l1", "observation_type": "phase",
                 "prefit_m": 0.2, "postfit_m": 5.0, "sigma_m": 0.01,
                 "elevation_deg": 10.0, "adopted": "true", "reason": ""},
                {"source_id": "TEST", "epoch_utc": "2026-07-27T00:00:02.000Z",
                 "satellite": "G02", "signal": "c1", "observation_type": "code",
                 "prefit_m": 2.0, "postfit_m": 0.0, "sigma_m": 1.0,
                 "elevation_deg": 20.0, "adopted": "false", "reason": "rejected"},
            ])
        return directory

    def test_ppp_cli_outputs_full_m5_contract(self) -> None:
        first = self.write_dataset("first.json", 0.0)
        second = self.write_dataset("second.json", 0.01)
        residuals = self.write_residuals()
        config = {
            "analysis": "ppp", "output_directory": "results",
            "datasets": [
                {"id": "first", "algorithm_label": "BIE", "path": first.name,
                 "reference_ecef_m": [6378137.0, 0.0, 0.0],
                 "residual_directory": "telemetry", "residual_source_id": "TEST"},
                {"id": "second", "algorithm_label": "LAMBDA", "path": second.name,
                 "reference_ecef_m": [6378137.0, 0.0, 0.0]},
            ],
            "options": {"horizontal_threshold_m": 0.1, "vertical_threshold_m": 0.2,
                        "sustain_seconds": 2.0, "convergence_gap_factor": 1.5,
                        "bootstrap_samples": 200, "bootstrap_confidence": 0.95},
        }
        config_path = self.directory / "config.json"
        config_path.write_text(json.dumps(config), encoding="utf-8")
        stderr = io.StringIO()
        with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(stderr):
            code = cli_main(["ppp", "--config", str(config_path)])
        self.assertEqual(code, 0, stderr.getvalue())
        output = self.directory / "results"
        for name in (
            "ppp_epochs.csv", "ppp_common_epoch_differences.csv", "ppp_jump_events.csv",
            "ppp_fixed_segments.csv", "ppp_reconvergence_events.csv", "ppp_residuals.csv",
            "ppp_residual_summary.csv", "ppp_enu_fixing.png", "ppp_error_histograms.png",
            "ppp_ztd_diagnostics.png", "ppp_residual_diagnostics.png", "metrics.json",
            "report.html", "manifest.json",
        ):
            self.assertTrue((output / name).is_file(), name)
        metrics = json.loads((output / "metrics.json").read_text(encoding="utf-8"))
        self.assertEqual(metrics["common_epoch_count"], 6)
        self.assertEqual(len(metrics["paired_comparisons"]), 2)
        self.assertEqual(metrics["datasets"][0]["residual_count"], 3)
        self.assertEqual(metrics["datasets"][0]["reference_mode"], "static")
        self.assertEqual(metrics["datasets"][0]["full_fix_segment_count"], 1)
        self.assertAlmostEqual(metrics["datasets"][0]["north_m"]["mean"], 0.0225)
        self.assertAlmostEqual(
            metrics["datasets"][0]["stored_minus_computed_neu_m"]["north"]["max_abs"],
            0.0,
        )
        self.assertEqual(metrics["datasets"][0]["nees"]["count"], 6)
        self.assertAlmostEqual(metrics["datasets"][0]["residual_nis"]["mean"], 200.5)
        with (output / "ppp_residuals.csv").open(encoding="utf-8", newline="") as stream:
            residual_rows = list(csv.DictReader(stream))
        self.assertIn("normalized_prefit", residual_rows[0])
        self.assertIn("ppp_error_histograms", (output / "report.html").read_text(encoding="utf-8"))

    def test_velocity_and_trajectory_reference(self) -> None:
        config_path = self.directory / "config.json"
        config_path.write_text("{}", encoding="utf-8")
        velocity = load_reference({
            "reference_ecef_m": [1.0, 2.0, 3.0],
            "reference_velocity_ecef_mps": [1.0, 0.0, 0.0],
            "reference_epoch_utc": "2026-07-27T00:00:00.000Z",
        }, config_path)
        np.testing.assert_allclose(velocity.at(1785110401_000_000_000), [2.0, 2.0, 3.0])
        trajectory = self.directory / "truth.csv"
        trajectory.write_text(
            "epoch_utc,x_m,y_m,z_m\n"
            "2026-07-27T00:00:00.000Z,1,2,3\n"
            "2026-07-27T00:00:10.000Z,11,2,3\n", encoding="utf-8"
        )
        provider = load_reference({"reference_ecef_m": [1, 2, 3], "truth_trajectory": "truth.csv"}, config_path)
        np.testing.assert_allclose(provider.at(1785110405_000_000_000), [6.0, 2.0, 3.0])
        self.assertTrue(np.all(np.isnan(provider.at(1785110411_000_000_000))))

    def test_paired_statistics_are_deterministic(self) -> None:
        self.assertLess(paired_sign_test_pvalue([1, 2, 3, 4]), 0.2)
        first = bootstrap_mean_ci([1.0, 2.0, 3.0], 100, 0.95, 7)
        second = bootstrap_mean_ci([1.0, 2.0, 3.0], 100, 0.95, 7)
        self.assertEqual(first, second)

    def test_final_residual_keys_must_be_unique(self) -> None:
        directory = self.write_residuals()
        path = directory / "ppp_residual_20260727.csv"
        lines = path.read_text(encoding="utf-8").splitlines()
        path.write_text("\n".join([*lines, lines[1]]) + "\n", encoding="utf-8")
        with self.assertRaisesRegex(ResidualError, "duplicate final-iteration residual"):
            read_residuals(directory)

    def test_ar_consistency_is_enforced(self) -> None:
        epoch = SimpleNamespace(
            fixed_count=1,
            eligible_count=0,
            used_count=4,
            satellite_sets=[list(range(4)), [], [1]],
            ar_status="partial_fix",
        )
        with self.assertRaisesRegex(ValueError, "fixed <= eligible <= used"):
            _validate_ar_epoch(epoch, 0)

    def test_jump_segments_and_reconvergence(self) -> None:
        rows = [
            {
                "epoch_ns": index * 1_000_000_000,
                "epoch_utc": f"2026-07-27T00:00:0{index}+00:00",
                "north_m": 1.0 if index == 2 else 0.0,
                "east_m": 0.0,
                "up_m": 0.0,
                "horizontal_m": 0.0,
                "any_fix": index in {0, 3, 4},
                "full_fix": index in {3, 4},
            }
            for index in range(5)
        ]
        jump_rows = [
            {
                "epoch_ns": index * 1_000_000_000,
                "epoch_utc": f"2026-07-27T00:00:{index:02d}+00:00",
                "north_m": 0.0 if index < 4 else 1.0,
                "east_m": 0.0,
                "up_m": 0.0,
            }
            for index in range(8)
        ]
        jumps, threshold = detect_robust_jumps(jump_rows, 8.0, 0.05)
        self.assertEqual(len(jumps), 1)
        self.assertEqual(threshold, 0.05)
        segments = fixed_segments(rows, "full_fix", 1.5)
        self.assertEqual((len(segments), segments[0]["epoch_count"]), (1, 2))
        events = loss_reconvergence_events(rows, 0.1, 0.2, 1.0, 1.5)
        self.assertEqual(len(events), 1)
        self.assertEqual(events[0]["refix_seconds"], 2.0)


if __name__ == "__main__":
    unittest.main()
