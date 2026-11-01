"""Tests observation availability and signal-quality analysis."""

from __future__ import annotations

import contextlib
import csv
import io
import json
import tempfile
import unittest
from pathlib import Path

from _support import RESEARCH_ROOT, rinex_header_line, rinex_observation  # noqa: F401
from rtppp_analysis.cli import main as cli_main


class ObservationCliTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.directory = Path(self.temporary.name)

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def write_rinex(self) -> Path:
        lines = [
            rinex_header_line("G    4 C1C L1C D1C S1C", "SYS / # / OBS TYPES"),
            rinex_header_line("", "END OF HEADER"),
        ]
        for second, satellites in ((0, ("G01", "G02")), (1, ("G01",))):
            lines.append(f"> 2026 07 27 00 00 {second:02d}.0000000  0  {len(satellites)}")
            for satellite in satellites:
                if satellite == "G02":
                    lines.append(satellite + rinex_observation(None) * 4)
                else:
                    lines.append(
                        satellite
                        + rinex_observation(20_000_000.0 + second)
                        + rinex_observation(110_000.0 + second, lli=second, ssi=7)
                        + rinex_observation(-1200.0 + second)
                        + rinex_observation(42.0 + second, ssi=8)
                    )
        lines.extend(
            [
                "> 2026 07 27 00 00 02.0000000  4  1",
                "ANTENNA MOVED",
                "> 2026 07 27 00 00 03.0000000  0  1",
                "G01"
                + rinex_observation(20_000_003.0)
                + rinex_observation(110_003.0, ssi=7)
                + rinex_observation(-1197.0)
                + rinex_observation(45.0, ssi=8),
            ]
        )
        path = self.directory / "observations.rnx"
        path.write_text("\n".join(lines) + "\n", encoding="utf-8")
        return path

    def test_observation_cli_writes_complete_schema_valid_outputs(self) -> None:
        self.write_rinex()
        config = {
            "analysis": "observations",
            "output_directory": "results",
            "datasets": [
                {
                    "id": "test-data",
                    "station": "TEST",
                    "rinex": "observations.rnx",
                    "time_scale": "UTC",
                }
            ],
            "options": {"expected_interval_seconds": 1.0, "gap_factor": 1.5},
        }
        config_path = self.directory / "observations.json"
        config_path.write_text(json.dumps(config), encoding="utf-8")
        stderr = io.StringIO()
        with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(stderr):
            exit_code = cli_main(["observations", "--config", str(config_path)])
        self.assertEqual(exit_code, 0, stderr.getvalue())

        output = self.directory / "results"
        for name in (
            "observation_epochs.csv",
            "observation_values.csv",
            "observation_events.csv",
            "observation_gaps.csv",
            "snr_values.csv",
            "signal_summary.csv",
            "snr_satellite_summary.csv",
            "msm_telemetry.csv",
            "ssr_telemetry.csv",
            "reception_events.csv",
            "lock_arcs.csv",
            "cycle_slips.csv",
            "half_cycle_events.csv",
            "snr_elevation_summary.csv",
            "metrics.json",
            "observations.png",
            "observation_signal_coverage.png",
            "observation_snr_histograms.png",
            "report.html",
            "manifest.json",
        ):
            with self.subTest(name=name):
                self.assertTrue((output / name).is_file(), name)

        metrics_text = (output / "metrics.json").read_text(encoding="utf-8")
        self.assertNotIn("NaN", metrics_text)
        metrics = json.loads(metrics_text)
        dataset = metrics["datasets"][0]
        self.assertEqual(dataset["epoch_count"], 3)
        self.assertEqual(dataset["event_epoch_count"], 1)
        self.assertEqual(dataset["satellite_record_count"], 4)
        self.assertEqual(dataset["observation_value_count"], 12)
        self.assertEqual(dataset["unique_satellite_count"], 2)
        self.assertEqual(
            dataset["system_satellite_counts"], {"C": 0, "E": 0, "G": 2, "R": 0}
        )
        self.assertEqual(dataset["gap_count"], 1)
        self.assertEqual(dataset["estimated_missing_epoch_count"], 1)
        c1c = next(
            row for row in metrics["signal_groups"] if row["observation_type"] == "C1C"
        )
        self.assertEqual(c1c["value_count"], 3)
        self.assertEqual(c1c["possible_value_count"], 4)
        self.assertEqual(c1c["value_coverage_rate"], 0.75)
        self.assertEqual(c1c["epoch_coverage_rate"], 1.0)
        self.assertEqual(c1c["satellite_coverage_rate"], 0.5)
        self.assertFalse(dataset["ssr_reception_analysis_available"])
        self.assertFalse(dataset["snr_elevation_analysis_available"])

    def test_observation_cli_integrates_structured_telemetry(self) -> None:
        self.write_rinex()
        telemetry = self.directory / "telemetry"
        telemetry.mkdir()
        msm_rows = []
        for second, lock, half, elevation in (
            (0, 10.0, 0, 5.0), (1, 11.0, 0, 15.0),
            (2, 1.0, 0, 25.0), (3, 2.0, 1, 85.0),
        ):
            utc = f"2026-07-27T00:00:{second:02d}.000Z"
            msm_rows.append(
                {"source_id": "OBS", "received_utc": utc, "epoch_utc": utc,
                 "station_id": 100, "system": "G", "satellite": "G01", "signal": "1C",
                 "has_lock_time": "true", "lock_time_seconds": lock,
                 "has_half_cycle": "true", "half_cycle_ambiguity": half,
                 "has_cnr": "true", "cnr_dbhz": 40 + second, "elevation_deg": elevation}
            )
        for name, rows in (
            ("msm_20260727.csv", msm_rows),
            ("ssr_20260727.csv", [{
                "source_id": "SSR", "received_utc": "2026-07-27T00:00:03.000Z",
                "epoch_utc": "2026-07-27T00:00:02.000Z", "message_type": 1057,
                "kind": "orbit", "system": "G", "satellite": "G01", "age_seconds": 1,
                "adopted": "false", "reason": "iod_mismatch"}]),
            ("reception_20260727.csv", [{
                "source_id": "SSR", "received_utc": "2026-07-27T00:00:03.000Z",
                "message_family": "SSR", "message_type": 1057, "system": "G",
                "status": "received", "reason": "", "detail": "one satellite"}, {
                "source_id": "SSR", "received_utc": "2026-07-27T00:00:04.000Z",
                "message_family": "STREAM", "message_type": 0, "system": "",
                "status": "reconnecting", "reason": "stream_interruption",
                "detail": "connection reset"}]),
        ):
            with (telemetry / name).open("w", encoding="utf-8", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
                writer.writeheader()
                writer.writerows(rows)
        config = {
            "analysis": "observations", "output_directory": "telemetry-results",
            "datasets": [{"id": "telemetry-data", "station": "TEST",
                          "rinex": "observations.rnx", "time_scale": "UTC",
                          "telemetry_directory": "telemetry"}],
            "options": {"expected_interval_seconds": 1.0, "gap_factor": 1.5,
                        "write_parquet": False},
        }
        config_path = self.directory / "telemetry-config.json"
        config_path.write_text(json.dumps(config), encoding="utf-8")
        stderr = io.StringIO()
        with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(stderr):
            exit_code = cli_main(["observations", "--config", str(config_path)])
        self.assertEqual(exit_code, 0, stderr.getvalue())
        output = self.directory / "telemetry-results"
        metrics = json.loads((output / "metrics.json").read_text(encoding="utf-8"))
        dataset = metrics["datasets"][0]
        self.assertTrue(dataset["telemetry_analysis_available"])
        self.assertTrue(dataset["ssr_reception_analysis_available"])
        self.assertTrue(dataset["snr_elevation_analysis_available"])
        self.assertEqual(dataset["cycle_slip_count"], 2)
        self.assertEqual(dataset["half_cycle_event_count"], 1)
        self.assertEqual(dataset["reception_status_counts"], {"received": 1, "reconnecting": 1})
        self.assertEqual(dataset["reception_reason_counts"], {"stream_interruption": 1})
        self.assertEqual(dataset["stream_interruption_count"], 1)
        self.assertEqual(len(metrics["snr_elevation_groups"]), 4)
        self.assertEqual(len((output / "cycle_slips.csv").read_text(encoding="utf-8").splitlines()), 3)
        for name in (
            "observation_snr_by_elevation.png", "observation_tracking.png",
            "observation_reception_integrity.png",
        ):
            self.assertTrue((output / name).is_file(), name)
            self.assertIn(name, (output / "report.html").read_text(encoding="utf-8"))


if __name__ == "__main__":
    unittest.main()
