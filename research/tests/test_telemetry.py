"""Tests telemetry parsing, validation, and residual summary contracts."""

from __future__ import annotations

import csv
import tempfile
import unittest
from unittest.mock import patch
from pathlib import Path

from _support import RESEARCH_ROOT  # noqa: F401
from rtppp_analysis.observations.telemetry import (
    TelemetryError,
    analyze_tracking,
    elevation_snr_summary,
    read_telemetry,
    write_partitioned_parquet,
)


class TelemetryTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.directory = Path(self.temporary.name)

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def write_csv(self, name: str, rows: list[dict[str, object]]) -> None:
        path = self.directory / name
        with path.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)

    def msm_row(
        self, second: int, lock: float, half: int, elevation: float, source: str = "OBS"
    ) -> dict[str, object]:
        utc = f"2026-07-27T00:00:{second:02d}.000Z"
        return {
            "source_id": source,
            "received_utc": utc,
            "epoch_utc": utc,
            "station_id": 100,
            "system": "G",
            "satellite": "G01",
            "signal": "1C",
            "has_lock_time": "true",
            "lock_time_seconds": lock,
            "has_half_cycle": "true",
            "half_cycle_ambiguity": half,
            "has_cnr": "true",
            "cnr_dbhz": 40 + second,
            "elevation_deg": elevation,
        }

    def write_complete_telemetry(self) -> None:
        self.write_csv(
            "msm_20260727.csv",
            [
                self.msm_row(0, 10.0, 0, 5.0),
                self.msm_row(1, 11.0, 0, 15.0),
                self.msm_row(2, 1.0, 0, 25.0),
                self.msm_row(3, 2.0, 1, 85.0),
            ],
        )
        self.write_csv(
            "ssr_20260727.csv",
            [{
                "source_id": "SSR", "received_utc": "2026-07-27T00:00:03.000Z",
                "epoch_utc": "2026-07-27T00:00:02.000Z", "message_type": 1057,
                "kind": "orbit", "system": "G", "satellite": "G01", "age_seconds": 1.0,
                "adopted": "false", "reason": "iod_mismatch",
            }],
        )
        self.write_csv(
            "reception_20260727.csv",
            [{
                "source_id": "SSR", "received_utc": "2026-07-27T00:00:03.000Z",
                "message_family": "SSR", "message_type": 1057, "system": "G",
                "status": "received", "reason": "", "detail": "kind=orbit satellites=1",
            }],
        )

    def test_read_tracking_slips_half_cycle_and_elevation_bins(self) -> None:
        self.write_complete_telemetry()
        telemetry = read_telemetry(self.directory)
        arcs, slips, half_cycles = analyze_tracking(telemetry["msm"], 1.0, 1.5)
        self.assertEqual(len(arcs), 3)
        self.assertEqual([row["evidence"] for row in slips], ["lock_time_reset", "half_cycle_change"])
        self.assertEqual(len(half_cycles), 1)
        bins = elevation_snr_summary(telemetry["msm"])
        self.assertEqual([row["elevation_min_deg"] for row in bins], [0, 10, 20, 80])
        self.assertEqual(telemetry["ssr"][0]["reason"], "iod_mismatch")

    def test_source_filter_and_invalid_reason(self) -> None:
        self.write_complete_telemetry()
        self.assertEqual(len(read_telemetry(self.directory, "OBS")["msm"]), 4)
        self.assertEqual(read_telemetry(self.directory, "OTHER")["msm"], [])
        path = self.directory / "ssr_20260727.csv"
        text = path.read_text(encoding="utf-8").replace("iod_mismatch", "packet_loss")
        path.write_text(text, encoding="utf-8")
        with self.assertRaisesRegex(TelemetryError, "invalid SSR reason"):
            read_telemetry(self.directory)

    def test_parquet_dependency_failure_is_explicit(self) -> None:
        real_import = __import__

        def missing_pandas(name: str, *args: object, **kwargs: object) -> object:
            if name == "pandas":
                raise ImportError("simulated")
            return real_import(name, *args, **kwargs)

        with patch("builtins.__import__", side_effect=missing_pandas):
            with self.assertRaisesRegex(RuntimeError, "full.*pandas and pyarrow"):
                write_partitioned_parquet(self.directory / "parquet", "TEST", {"msm": []})

    def test_zstd_parquet_is_partitioned_by_date_station_and_source(self) -> None:
        try:
            import pandas  # noqa: F401
            import pyarrow.parquet as parquet
        except ImportError:
            self.skipTest("pandas/pyarrow full extra is not installed")
        self.write_complete_telemetry()
        rows = read_telemetry(self.directory)
        paths = write_partitioned_parquet(self.directory / "parquet", "TEST", rows)
        msm_path = next(path for path in paths if path.name == "msm.parquet")
        self.assertIn("date=2026-07-27", msm_path.as_posix())
        self.assertIn("station=TEST", msm_path.as_posix())
        self.assertIn("source=OBS", msm_path.as_posix())
        compression = parquet.ParquetFile(msm_path).metadata.row_group(0).column(0).compression
        self.assertEqual(compression, "ZSTD")


if __name__ == "__main__":
    unittest.main()
