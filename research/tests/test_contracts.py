"""Tests public JSON schemas and telemetry/data-contract validation."""

from __future__ import annotations

import copy
import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from _support import RESEARCH_ROOT, compact_root
from rtppp_analysis.config import ConfigError, ContractError
from rtppp_analysis.contracts import validate_compact, validate_config, validate_metrics
from rtppp_analysis.io.tables import json_value, write_json
from rtppp_analysis.models import Product
from rtppp_analysis.orbit.compare import _metadata_warnings


class ConfigurationContractTests(unittest.TestCase):
    def load_example(self, name: str) -> dict:
        return json.loads((RESEARCH_ROOT / "configs" / name).read_text(encoding="utf-8"))

    def test_all_example_configurations_validate(self) -> None:
        for command, name in (
            ("orbit", "orbit_compare.example.json"),
            ("observations", "observation_analysis.example.json"),
            ("ppp", "ppp_analysis.example.json"),
        ):
            with self.subTest(command=command):
                validate_config(command, self.load_example(name))

    def test_unknown_configuration_field_is_rejected(self) -> None:
        config = self.load_example("ppp_analysis.example.json")
        config["unexpected"] = True
        with self.assertRaisesRegex(ConfigError, r"\$.*unexpected"):
            validate_config("ppp", config)

    def test_missing_reference_frame_is_rejected(self) -> None:
        config = self.load_example("orbit_compare.example.json")
        del config["reference"]["reference_frame"]
        with self.assertRaises(ConfigError):
            validate_config("orbit", config)

    def test_command_and_analysis_must_match(self) -> None:
        config = self.load_example("ppp_analysis.example.json")
        with self.assertRaises(ConfigError):
            validate_config("observations", config)

    def test_duplicate_dataset_ids_are_rejected(self) -> None:
        config = self.load_example("ppp_analysis.example.json")
        config["datasets"][1]["id"] = config["datasets"][0]["id"]
        with self.assertRaisesRegex(ConfigError, "duplicate id"):
            validate_config("ppp", config)

    def test_empty_or_padded_id_is_rejected(self) -> None:
        for identifier in ("   ", " station_bjfs "):
            with self.subTest(identifier=identifier):
                config = self.load_example("observation_analysis.example.json")
                config["datasets"][0]["id"] = identifier
                with self.assertRaises(ConfigError):
                    validate_config("observations", config)

    def test_invalid_enum_is_rejected(self) -> None:
        config = self.load_example("orbit_compare.example.json")
        config["reference"]["time_scale"] = "TAI"
        with self.assertRaises(ConfigError):
            validate_config("orbit", config)

    def test_reference_ecef_requires_exactly_three_values(self) -> None:
        config = self.load_example("ppp_analysis.example.json")
        config["datasets"][0]["reference_ecef_m"] = [1.0, 2.0]
        with self.assertRaises(ConfigError):
            validate_config("ppp", config)

    def test_ppp_reference_modes_cannot_conflict(self) -> None:
        config = self.load_example("ppp_analysis.example.json")
        config["datasets"][0]["truth_trajectory"] = "truth.csv"
        config["datasets"][0]["reference_velocity_ecef_mps"] = [0.0, 0.0, 0.0]
        config["datasets"][0]["reference_epoch_utc"] = "2026-07-27T00:00:00Z"
        with self.assertRaises(ConfigError):
            validate_config("ppp", config)


class DataContractTests(unittest.TestCase):
    def test_packaged_schema_copies_match_public_contracts(self) -> None:
        public_directory = RESEARCH_ROOT / "schemas"
        packaged_directory = RESEARCH_ROOT / "src" / "rtppp_analysis" / "schemas"
        public_names = {path.name for path in public_directory.glob("*.schema.json")}
        packaged_names = {path.name for path in packaged_directory.glob("*.schema.json")}
        self.assertEqual(packaged_names, public_names)
        for name in public_names:
            self.assertEqual(
                (packaged_directory / name).read_bytes(),
                (public_directory / name).read_bytes(),
                name,
            )

    def test_compact_v1_contract_accepts_writer_layout(self) -> None:
        validate_compact(compact_root())

    def test_compact_solution_must_have_exactly_eleven_items(self) -> None:
        value = compact_root()
        value["solutions"][0].pop()
        with self.assertRaises(ContractError):
            validate_compact(value)

    def test_compact_enum_index_bounds_are_enforced(self) -> None:
        value = compact_root()
        value["solutions"][0][1] = 3
        with self.assertRaises(ContractError):
            validate_compact(value)

    def test_compact_integer_counts_are_enforced(self) -> None:
        for location in ("satellite summary", "BIE candidate count"):
            with self.subTest(location=location):
                value = compact_root()
                if location == "satellite summary":
                    value["solutions"][0][7][0] = 8.5
                else:
                    value["solutions"][0][9][0] = 8.5
                with self.assertRaises(ContractError):
                    validate_compact(value)

    def test_compact_time_base_must_be_consistent_utc_midnight(self) -> None:
        value = compact_root()
        value["meta"]["timeBase"]["gpsWeek"] += 1
        with self.assertRaisesRegex(ContractError, r"\$\.meta\.timeBase\.gpsWeek"):
            validate_compact(value)

    def test_non_finite_values_are_rejected_before_schema_validation(self) -> None:
        value = compact_root()
        value["solutions"][0][3][0] = float("nan")
        with self.assertRaisesRegex(ContractError, "non-finite"):
            validate_compact(value)

    def test_minimal_metrics_contracts(self) -> None:
        orbit = {
            "reference": "ref",
            "candidates": ["candidate"],
            "products": [
                {
                    "id": "ref",
                    "role": "reference",
                    "time_scale": "GPS",
                    "orbit_reference": "APC",
                    "reference_frame": "IGS20",
                },
                {
                    "id": "candidate",
                    "role": "candidate",
                    "time_scale": "UTC",
                    "orbit_reference": "UNKNOWN",
                    "reference_frame": "UNKNOWN",
                },
            ],
            "row_count": 0,
            "common_sample_count": 0,
            "metrics": [],
            "satellite_metrics": [],
            "coverage": [],
            "warnings": [],
        }
        observations = {
            "datasets": [],
            "signal_groups": [],
            "snr_satellite_groups": [],
            "snr_elevation_groups": [],
            "warnings": [],
        }
        ppp = {
            "datasets": [],
            "common_epoch_count": 0,
            "common_epoch_coverage": [],
            "paired_comparisons": [],
            "residual_groups": [],
            "convergence_definition": {
                "horizontal_threshold_m": 0.1,
                "vertical_threshold_m": 0.2,
                "sustain_seconds": 600.0,
            },
            "warnings": [],
        }
        validate_metrics("orbit_clock", orbit)
        validate_metrics("observations", observations)
        validate_metrics("ppp", ppp)

    def test_numpy_and_non_finite_values_normalize_to_standard_json(self) -> None:
        normalized = json_value(
            {
                "value": np.float64(1.5),
                "array": np.asarray([1, 2], dtype=np.int64),
                "nan": np.nan,
                "positive_infinity": np.inf,
                "negative_infinity": -np.inf,
            }
        )
        self.assertEqual(
            normalized,
            {
                "value": 1.5,
                "array": [1, 2],
                "nan": None,
                "positive_infinity": None,
                "negative_infinity": None,
            },
        )
        encoded = json.dumps(normalized, allow_nan=False)
        for token in ("NaN", "Infinity", "-Infinity"):
            self.assertNotIn(token, encoded)

        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "metrics.json"
            write_json(path, normalized)
            serialized = path.read_text(encoding="utf-8")
            for token in ("NaN", "Infinity", "-Infinity"):
                self.assertNotIn(token, serialized)
            self.assertEqual(json.loads(serialized), normalized)

    def test_metrics_coverage_rates_are_bounded(self) -> None:
        value = {
            "datasets": [],
            "common_epoch_count": 0,
            "common_epoch_coverage": [
                {
                    "dataset_id": "candidate",
                    "epoch_count": 1,
                    "common_epoch_count": 1,
                    "coverage_rate": 1.1,
                }
            ],
            "paired_comparisons": [],
            "residual_groups": [],
            "convergence_definition": {
                "horizontal_threshold_m": 0.1,
                "vertical_threshold_m": 0.2,
                "sustain_seconds": 600.0,
            },
            "warnings": [],
        }
        with self.assertRaises(ContractError):
            validate_metrics("ppp", value)


class OrbitMetadataTests(unittest.TestCase):
    def product(
        self,
        product_id: str,
        time_scale: str,
        orbit_reference: str,
        reference_frame: str,
    ) -> Product:
        return Product(product_id, time_scale, orbit_reference, reference_frame, {}, {}, [])

    def test_unknown_and_incompatible_metadata_produce_warnings(self) -> None:
        reference = self.product("reference", "GPS", "APC", "IGS20")
        candidate = self.product("candidate", "UTC", "UNKNOWN", "UNKNOWN")
        warnings = "\n".join(_metadata_warnings(candidate, reference))
        self.assertIn("incompatible time scales", warnings)
        self.assertIn("APC/CoM semantics are unknown", warnings)
        self.assertIn("reference frame is unknown", warnings)


if __name__ == "__main__":
    unittest.main()
