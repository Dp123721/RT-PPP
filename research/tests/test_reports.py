"""Tests report serialization and generated artifacts."""

from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from rtppp_analysis.reports.html import write_html_report


class HtmlReportTests(unittest.TestCase):
    def test_orbit_report_is_compact_and_keeps_core_sisre_metrics(self) -> None:
        summary = {
            "reference": "igs",
            "candidates": ["center_a"],
            "row_count": 100,
            "common_sample_count": 90,
            "metrics": [
                {
                    "scope": "all",
                    "candidate_id": "center_a",
                    "reference_id": "igs",
                    "system": "G",
                    "sample_count": 90,
                    "satellite_count": 12,
                    "orbit_3d_m_rms": 0.12,
                    "clock_datum_removed_m_rms": 0.08,
                    "sisre_m_rms": 0.14,
                    "sisre_m_coverage_rate": 0.95,
                    "radial_m_p95_abs": 99.0,
                }
            ],
            "satellite_metrics": [{"satellite": "G01", "very_large_detail": list(range(100))}],
            "coverage": [],
            "warnings": [],
        }
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "report.html"
            write_html_report(path, "Orbit and Clock Analysis", summary, [], [])
            content = path.read_text(encoding="utf-8")
        self.assertIn("SISRE RMS (m)", content)
        self.assertIn("0.14", content)
        self.assertNotIn("very_large_detail", content)
        self.assertNotIn("radial_m_p95_abs", content)
        self.assertNotIn('&quot;metrics&quot;', content)


if __name__ == "__main__":
    unittest.main()
