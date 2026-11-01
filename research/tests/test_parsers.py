"""Tests parsers for orbit, observation, and PPP source files."""

from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

import numpy as np

from _support import (
    compact_root,
    compact_solution,
    rinex_header_line,
    rinex_observation,
    write_json,
)
from rtppp_analysis.io.ppp_json import PppJsonError, read_ppp_json
from rtppp_analysis.io.rinex_obs import (
    RinexParseError,
    read_rinex_observation_file,
    read_rinex_observations,
)
from rtppp_analysis.io.sp3_clk import ProductParseError, load_product, read_clk, read_sp3


class DuplicateParserTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.directory = Path(self.temporary.name)

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def write_text(self, name: str, text: str) -> Path:
        path = self.directory / name
        path.write_text(text, encoding="utf-8")
        return path

    def test_duplicate_sp3_position_is_rejected(self) -> None:
        path = self.write_text(
            "duplicate.sp3",
            "*  2026 07 27 00 00 00.00000000\n"
            "PG01      1.0      2.0      3.0      4.0\n"
            "PG01      1.1      2.0      3.0      4.0\n",
        )
        with self.assertRaisesRegex(ProductParseError, "Duplicate SP3 P"):
            read_sp3(path, "UTC")

    def test_product_metadata_is_case_normalized(self) -> None:
        path = self.write_text(
            "metadata.sp3",
            "*  2026 07 27 00 00 00.00000000\n"
            "PG01      1.0      2.0      3.0      4.0\n",
        )
        product = load_product(
            {
                "id": "candidate",
                "sp3": path.name,
                "time_scale": "utc",
                "orbit_reference": "com",
                "reference_frame": "igs20",
            },
            self.directory,
        )
        self.assertEqual(product.time_scale, "UTC")
        self.assertEqual(product.orbit_reference, "COM")
        self.assertEqual(product.reference_frame, "IGS20")

    def test_sp3_position_velocity_clock_units_and_missing_markers(self) -> None:
        path = self.write_text(
            "units.sp3",
            "*  2026 07 27 00 00 00.00000000\n"
            "PG01      1.0      2.0      3.0      4.0\n"
            "VG01     10.0     20.0     30.0     40.0\n"
            "PG02      0.0      0.0      0.0 999999.999999\n"
            "VG02     10.0     20.0     30.0     40.0\n"
            "PG03      4.0      5.0      6.0 999999.999999\n",
        )
        parsed = read_sp3(path, "UTC")
        np.testing.assert_allclose(parsed["G01"][0].position_m, [1000.0, 2000.0, 3000.0])
        np.testing.assert_allclose(parsed["G01"][0].velocity_mps, [1.0, 2.0, 3.0])
        self.assertEqual(parsed["G01"][0].clock_s, 4e-6)
        self.assertNotIn("G02", parsed)
        self.assertTrue(np.isnan(parsed["G03"][0].clock_s))

    def test_clk_standard_and_rtppp_records_use_seconds(self) -> None:
        path = self.write_text(
            "units.clk",
            "END OF HEADER\n"
            "AS G01 2026 07 27 00 00 00.000000 1 1.25e-6\n"
            "AS G02 2026 07 27 00 00 00.000000 -2.50e-6\n",
        )
        parsed = read_clk(path, "UTC")
        self.assertEqual(parsed["G01"][0].clock_s, 1.25e-6)
        self.assertEqual(parsed["G02"][0].clock_s, -2.50e-6)

    def test_malformed_sp3_records_are_rejected(self) -> None:
        cases = {
            "before-epoch": "PG01 1.0 2.0 3.0 4.0\n",
            "short-record": "*  2026 07 27 00 00 00.0\nPG01 1.0 2.0\n",
            "non-finite": "*  2026 07 27 00 00 00.0\nPG01 nan 2.0 3.0 4.0\n",
            "velocity-without-position": "*  2026 07 27 00 00 00.0\nVG01 1.0 2.0 3.0 4.0\n",
            "no-epoch": "#dP synthetic header\nEOF\n",
        }
        for name, content in cases.items():
            with self.subTest(name=name):
                path = self.write_text(f"{name}.sp3", content)
                with self.assertRaises(ProductParseError):
                    read_sp3(path, "UTC")

    def test_malformed_clk_header_and_values_are_rejected(self) -> None:
        cases = {
            "incomplete-header": "AS G01 2026 07 27 00 00 00.0 1 1e-6\n",
            "short-record": "END OF HEADER\nAS G01 2026 07 27 00 00\n",
            "non-finite": "END OF HEADER\nAS G01 2026 07 27 00 00 00.0 nan\n",
            "bad-count": "END OF HEADER\nAS G01 2026 07 27 00 00 00.0 2 1e-6\n",
        }
        for name, content in cases.items():
            with self.subTest(name=name):
                path = self.write_text(f"{name}.clk", content)
                with self.assertRaises(ProductParseError):
                    read_clk(path, "UTC")

    def test_duplicate_sp3_velocity_is_rejected(self) -> None:
        path = self.write_text(
            "duplicate-v.sp3",
            "*  2026 07 27 00 00 00.00000000\n"
            "PG01      1.0      2.0      3.0      4.0\n"
            "VG01      1.0      2.0      3.0      0.0\n"
            "VG01      1.0      2.0      3.0      0.0\n",
        )
        with self.assertRaisesRegex(ProductParseError, "Duplicate SP3 V"):
            read_sp3(path, "UTC")

    def test_duplicate_clk_record_is_rejected(self) -> None:
        record = "AS G01 2026 07 27 00 00 00.000000 1 1.0e-6\n"
        path = self.write_text("duplicate.clk", "END OF HEADER\n" + record + record)
        with self.assertRaisesRegex(ProductParseError, "Duplicate CLK"):
            read_clk(path, "UTC")

    def rinex_header(self) -> str:
        return "\n".join(
            [
                rinex_header_line("G    1 C1C", "SYS / # / OBS TYPES"),
                rinex_header_line("R    1 C1C", "SYS / # / OBS TYPES"),
                rinex_header_line("", "END OF HEADER"),
            ]
        ) + "\n"

    def test_rinex_same_epoch_different_systems_is_valid(self) -> None:
        epoch = "> 2026 07 27 00 00 00.0000000  0  1\n"
        path = self.write_text(
            "multisystem.rnx",
            self.rinex_header()
            + epoch
            + "G01"
            + rinex_observation(20_000_000.0)
            + "\n"
            + epoch
            + "R01"
            + rinex_observation(21_000_000.0)
            + "\n",
        )
        observations = read_rinex_observations(path, "UTC")
        self.assertEqual(len(observations), 2)

    def test_duplicate_rinex_observation_is_rejected(self) -> None:
        epoch = "> 2026 07 27 00 00 00.0000000  0  1\n"
        record = "G01" + rinex_observation(20_000_000.0) + "\n"
        path = self.write_text("duplicate.rnx", self.rinex_header() + epoch + record + epoch + record)
        with self.assertRaisesRegex(RinexParseError, "Duplicate observation"):
            read_rinex_observations(path, "UTC")

    def test_rinex_multiline_header_satellite_continuation_empty_record_and_event(self) -> None:
        header = "\n".join(
            [
                rinex_header_line("G    5 C1C L1C D1C S1C", "SYS / # / OBS TYPES"),
                rinex_header_line("      C2W", "SYS / # / OBS TYPES"),
                rinex_header_line("", "END OF HEADER"),
            ]
        ) + "\n"
        epoch = "> 2026 07 27 00 00 00.0000000  0  2\n"
        first_line = "G01" + "".join(
            [
                rinex_observation(20_000_000.0),
                rinex_observation(110_000.0, lli=1, ssi=7),
                rinex_observation(-1200.0),
                rinex_observation(45.0, ssi=8),
            ]
        )
        continuation = "   " + rinex_observation(20_000_010.0)
        empty_satellite = "G02" + rinex_observation(None) * 5
        event = "> 2026 07 27 00 00 01.0000000  4  1\nEVENT COMMENT\n"
        path = self.write_text(
            "multiline.rnx",
            header
            + epoch
            + first_line
            + "\n"
            + continuation
            + "\n"
            + empty_satellite
            + "\n"
            + event,
        )
        parsed = read_rinex_observation_file(path, "UTC")
        self.assertEqual(parsed.observation_types["G"], ["C1C", "L1C", "D1C", "S1C", "C2W"])
        self.assertEqual(len(parsed.observations), 5)
        self.assertEqual(len(parsed.epoch_satellites), 1)
        self.assertEqual(next(iter(parsed.epoch_satellites.values())), {"G01", "G02"})
        l1c = next(item for item in parsed.observations if item.observation_type == "L1C")
        self.assertEqual((l1c.lli, l1c.ssi), (1, 7))
        self.assertEqual(parsed.event_epochs[0][1:], (4, 1))

    def test_rinex_header_count_and_truncated_epoch_are_rejected(self) -> None:
        bad_header = "\n".join(
            [
                rinex_header_line("G    2 C1C", "SYS / # / OBS TYPES"),
                rinex_header_line("", "END OF HEADER"),
            ]
        ) + "\n"
        with self.assertRaisesRegex(RinexParseError, "count mismatch"):
            read_rinex_observations(self.write_text("bad-header.rnx", bad_header), "UTC")

        header = self.rinex_header()
        truncated = header + "> 2026 07 27 00 00 00.0000000  0  1\n"
        with self.assertRaisesRegex(RinexParseError, "Truncated observation epoch"):
            read_rinex_observations(self.write_text("truncated.rnx", truncated), "UTC")

    def test_duplicate_ppp_epoch_is_rejected(self) -> None:
        path = self.directory / "duplicate.json"
        write_json(path, compact_root([compact_solution(1000), compact_solution(1000)]))
        with self.assertRaisesRegex(PppJsonError, "Duplicate PPP epoch"):
            read_ppp_json(path)

    def test_part_file_is_rejected(self) -> None:
        path = self.directory / "runtime.json.part"
        write_json(path, compact_root())
        with self.assertRaisesRegex(PppJsonError, r"\.part"):
            read_ppp_json(path)

    def test_invalid_compact_array_is_rejected(self) -> None:
        value = compact_root()
        value["solutions"][0][3] = [1.0, 2.0]
        path = self.directory / "invalid.json"
        write_json(path, value)
        with self.assertRaisesRegex(PppJsonError, "contract violation"):
            read_ppp_json(path)

    def test_invalid_compact_schema_and_version_are_rejected(self) -> None:
        for field, invalid in (("schema", "other.compact"), ("version", 2)):
            with self.subTest(field=field):
                value = compact_root()
                value["meta"][field] = invalid
                path = self.directory / f"invalid-{field}.json"
                write_json(path, value)
                with self.assertRaisesRegex(PppJsonError, "contract violation"):
                    read_ppp_json(path)


if __name__ == "__main__":
    unittest.main()
