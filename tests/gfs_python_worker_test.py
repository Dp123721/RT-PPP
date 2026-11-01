"""Checks the GFS download worker's request, cache, and decoding contract."""

import datetime as dt
import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path
from urllib.parse import parse_qs, urlsplit
from unittest.mock import patch


MODULE_PATH = Path(__file__).resolve().parents[1] / "gfs_python" / "gfs_worker.py"
SPEC = importlib.util.spec_from_file_location("gfs_worker_under_test", MODULE_PATH)
WORKER = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = WORKER
SPEC.loader.exec_module(WORKER)


class GfsWorkerFallbackTest(unittest.TestCase):
    def test_filter_url_enables_spatial_subset(self):
        requested = WORKER.parse_utc("2026-09-02T09:00:00Z")
        address = WORKER.product_address(requested)
        url = WORKER.build_url("", address, 39.7, 116.5)
        query = parse_qs(urlsplit(url).query, keep_blank_values=True)

        self.assertIn("subregion", query)
        self.assertEqual(query["subregion"], [""])
        self.assertEqual(query["leftlon"], ["116.000"])
        self.assertEqual(query["rightlon"], ["117.000"])
        self.assertEqual(query["bottomlat"], ["39.200"])
        self.assertEqual(query["toplat"], ["40.200"])

    def test_previous_cycle_preserves_effective_hour(self):
        requested = WORKER.parse_utc("2026-09-02T09:00:00Z")
        nominal = WORKER.product_address(requested)
        previous = WORKER.product_address(
            requested, nominal["cycle"] - dt.timedelta(hours=6)
        )
        self.assertEqual((nominal["cycleText"], nominal["leadHour"]), ("2026090206", 3))
        self.assertEqual((previous["cycleText"], previous["leadHour"]), ("2026090200", 9))

    def test_404_selects_previous_cycle(self):
        requested = WORKER.parse_utc("2026-09-02T09:00:00Z")
        profile = {"validTime": "2026-09-02T09:00:00Z", "levels": [{"pressurePa": 100000}]}
        with tempfile.TemporaryDirectory() as directory:
            request = {
                "requestedUtc": requested.isoformat(),
                "latitudeDeg": 39.7,
                "longitudeDeg": 116.5,
                "cacheDirectory": directory,
            }
            calls = []

            def fake_download(_request, address):
                calls.append((address["cycleText"], address["leadHour"]))
                if len(calls) == 1:
                    raise WORKER.ProductNotFound("GFS HTTP 404: Not Found")
                return Path(directory) / "fallback.grib2", "download"

            with patch.object(WORKER, "download_to_cache", side_effect=fake_download), \
                    patch.object(WORKER, "decode_profile", return_value=profile), \
                    patch.object(WORKER, "require_eccodes", return_value=object()):
                result = WORKER.process(request)

        self.assertTrue(result["ok"])
        self.assertTrue(result["fallbackUsed"])
        self.assertEqual(result["selection"], "previous_cycle")
        self.assertEqual(result["productCycle"], "2026090200")
        self.assertEqual(result["forecastLeadSec"], 9 * 3600)
        self.assertEqual(calls, [("2026090206", 3), ("2026090200", 9)])
        self.assertEqual(result["attempts"][0]["cycle"], "2026090206")

    def test_non_404_error_is_not_hidden(self):
        request = {
            "requestedUtc": "2026-09-02T09:00:00Z",
            "latitudeDeg": 39.7,
            "longitudeDeg": 116.5,
            "cacheDirectory": ".",
        }
        with patch.object(WORKER, "download_to_cache", side_effect=RuntimeError("network down")), \
                patch.object(WORKER, "require_eccodes", return_value=object()):
            with self.assertRaisesRegex(RuntimeError, "network down"):
                WORKER.process(request)

    def test_transient_network_error_tries_previous_cycle(self):
        requested = WORKER.parse_utc("2026-09-02T09:00:00Z")
        profile = {"validTime": "2026-09-02T09:00:00Z",
                   "levels": [{"pressurePa": 100000}]}
        with tempfile.TemporaryDirectory() as directory:
            request = {
                "requestedUtc": requested.isoformat(),
                "latitudeDeg": 39.7,
                "longitudeDeg": 116.5,
                "cacheDirectory": directory,
            }
            calls = []

            def fake_download(_request, address):
                calls.append((address["cycleText"], address["leadHour"]))
                if len(calls) == 1:
                    raise WORKER.ProductUnavailable("GFS download timeout after 20s")
                return Path(directory) / "fallback.grib2", "download"

            with patch.object(WORKER, "download_to_cache", side_effect=fake_download), \
                    patch.object(WORKER, "decode_profile", return_value=profile), \
                    patch.object(WORKER, "require_eccodes", return_value=object()):
                result = WORKER.process(request)

        self.assertTrue(result["ok"])
        self.assertTrue(result["fallbackUsed"])
        self.assertEqual(result["selection"], "previous_cycle")
        self.assertEqual(calls, [("2026090206", 3), ("2026090200", 9)])


if __name__ == "__main__":
    unittest.main()
