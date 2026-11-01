#!/usr/bin/env python3
"""Download and decode one GFS GRIB2 profile for RT-PPP.

The process boundary is intentional: the Qt application does not depend on
the ecCodes C library.  One JSON request is read from stdin and one JSON
response is written to stdout.  Human-readable diagnostics go to stderr.
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import math
import os
from pathlib import Path
import sys
import tempfile
import time
from typing import Any
import socket
from urllib.error import HTTPError, URLError
from urllib.parse import urlencode
from urllib.request import Request, urlopen


DEFAULT_BASE_URL = "https://nomads.ncep.noaa.gov/cgi-bin/filter_gfs_0p25.pl"
PRESSURE_LEVELS = (
    1000, 975, 950, 925, 900, 850, 800, 750, 700, 650, 600, 550,
    500, 450, 400, 350, 300, 250, 200, 150, 100,
)


class ProductNotFound(RuntimeError):
    """The requested cycle/lead is not published at the product server."""


class ProductUnavailable(RuntimeError):
    """The product request failed transiently and an older cycle may work."""


def diagnostic(message: str) -> None:
    print(f"GFS-PY: {message}", file=sys.stderr, flush=True)


def require_eccodes() -> Any:
    try:
        import eccodes
    except Exception as exc:
        runtime_dir = Path(__file__).resolve().parent
        raise RuntimeError(
            f"Python ecCodes import failed: {exc}. Initialize the private GFS runtime "
            f"by running bootstrap.ps1 in {runtime_dir} (or bootstrap.sh on Linux)."
        ) from exc
    return eccodes


def parse_utc(value: str) -> dt.datetime:
    text = value.strip()
    if text.endswith("Z"):
        text = text[:-1] + "+00:00"
    parsed = dt.datetime.fromisoformat(text)
    if parsed.tzinfo is None:
        parsed = parsed.replace(tzinfo=dt.timezone.utc)
    return parsed.astimezone(dt.timezone.utc)


def product_address(requested: dt.datetime, cycle: dt.datetime | None = None) -> dict[str, Any]:
    if cycle is None:
        cycle_hour = (requested.hour // 6) * 6
        cycle = requested.replace(hour=cycle_hour, minute=0, second=0, microsecond=0)
    lead = max(0, int((requested - cycle).total_seconds() + 1800) // 3600)
    return {
        "valid": requested,
        "cycle": cycle,
        "cycleText": cycle.strftime("%Y%m%d%H"),
        "leadHour": lead,
        "filename": f"gfs.t{cycle:%H}z.pgrb2.0p25.f{lead:03d}",
    }


def build_url(base: str, address: dict[str, Any], lat: float, lon: float) -> str:
    half = 0.5
    params: list[tuple[str, str]] = [
        ("file", address["filename"]),
        ("dir", f"/gfs.{address['cycle']:%Y%m%d}/{address['cycle']:%H}/atmos"),
        # NOMADS uses the presence of this parameter to enable the geographic
        # subset controls below. Without it, the CGI may return the selected
        # fields for the full global grid despite receiving the bounds.
        ("subregion", ""),
        ("var_TMP", "on"),
        ("var_SPFH", "on"),
        ("var_HGT", "on"),
    ]
    params.extend((f"lev_{level}_mb", "on") for level in PRESSURE_LEVELS)
    params.extend(
        (
            ("leftlon", f"{lon - half:.3f}"),
            ("rightlon", f"{lon + half:.3f}"),
            ("toplat", f"{lat + half:.3f}"),
            ("bottomlat", f"{lat - half:.3f}"),
        )
    )
    return f"{base.strip() or DEFAULT_BASE_URL}?{urlencode(params)}"


def cache_path(cache_dir: Path, address: dict[str, Any], lat: float, lon: float) -> Path:
    key = f"{address['cycleText']}_{address['leadHour']:03d}_{lat:.2f}_{lon:.2f}"
    return cache_dir / f"{key.replace('-', 'm').replace('.', '_')}.grib2"


def download_to_cache(request: dict[str, Any], address: dict[str, Any]) -> tuple[Path, str]:
    cache_text = str(request.get("cacheDirectory", "")).strip()
    if not cache_text:
        raise RuntimeError("GFS cache directory is empty")
    cache_dir = Path(cache_text).expanduser()
    cache_dir.mkdir(parents=True, exist_ok=True)
    lat = float(request["latitudeDeg"])
    lon = float(request["longitudeDeg"])
    path = cache_path(cache_dir, address, lat, lon)
    max_age = int(request.get("maxAgeSec", 0))
    if path.is_file() and path.stat().st_size > 64:
        age = max(0.0, dt.datetime.now(dt.timezone.utc).timestamp() - path.stat().st_mtime)
        if max_age <= 0 or age <= max_age:
            diagnostic(f"cache hit path={path} bytes={path.stat().st_size} ageSec={age:.0f}")
            return path, "cache"

    url = build_url(str(request.get("downloadBaseUrl", "")), address, lat, lon)
    diagnostic(f"download start validUtc={address['valid'].isoformat()} cycle={address['cycleText']} "
               f"leadHour={address['leadHour']} lat={lat:.5f} lon={lon:.5f} url={url}")
    requested_timeout = max(1, int(request.get("timeoutSec", 90)))
    # The NOMADS filter can spend more than a short UI timeout preparing the
    # subset before the first byte.  Keep a floor so an old configuration with
    # 20 seconds does not turn a valid but slow response into a false failure.
    timeout = max(30, requested_timeout)
    try:
        http_request = Request(url, headers={"User-Agent": "RT-PPP-GFS-Python/1.0"})
        request_started = time.monotonic()
        diagnostic("waiting for GFS HTTP response")
        with urlopen(http_request, timeout=timeout) as response:
            # Read in bounded chunks. urllib applies ``timeout`` to the
            # underlying socket, so this is an inactivity timeout for each
            # read rather than an incorrect total-size limit. A 50-60 MB
            # subset may legitimately take longer than 20 seconds to arrive.
            body = bytearray()
            content_length_text = response.headers.get("Content-Length", "")
            try:
                content_length = int(content_length_text)
            except (TypeError, ValueError):
                content_length = 0
            diagnostic(f"GFS HTTP response received status={getattr(response, 'status', 200)} "
                       f"contentLength={content_length or 'unknown'} "
                       f"responseSec={time.monotonic() - request_started:.1f}")
            last_report = request_started
            last_report_bytes = 0
            # HTTPResponse.read(size) may wait until the requested size is
            # available for a chunked NOMADS response. read1() returns the
            # bytes currently available, so the GUI can report real progress
            # while a large filtered GRIB2 product is still arriving.
            read_chunk = getattr(response, "read1", None)
            if not callable(read_chunk):
                read_chunk = response.read
            while True:
                chunk = read_chunk(512 * 1024)
                if not chunk:
                    break
                body.extend(chunk)
                now = time.monotonic()
                elapsed = max(now - request_started, 1.0e-6)
                if (now - last_report >= 2.0 or
                        len(body) - last_report_bytes >= 4 * 1024 * 1024):
                    percent = (100.0 * len(body) / content_length
                               if content_length > 0 else None)
                    if percent is None:
                        diagnostic(f"GFS download progress bytes={len(body)} "
                                   f"rateKBps={len(body) / elapsed / 1024.0:.1f} "
                                   f"elapsedSec={elapsed:.1f}")
                    else:
                        diagnostic(f"GFS download progress bytes={len(body)}/{content_length} "
                                   f"percent={percent:.1f} rateKBps={len(body) / elapsed / 1024.0:.1f} "
                                   f"elapsedSec={elapsed:.1f}")
                    last_report = now
                    last_report_bytes = len(body)
            status = getattr(response, "status", 200)
            content_type = response.headers.get("Content-Type", "")
    except HTTPError as exc:
        if exc.code == 404:
            raise ProductNotFound(f"GFS HTTP 404: {exc.reason}") from exc
        raise RuntimeError(f"GFS HTTP {exc.code}: {exc.reason}") from exc
    except URLError as exc:
        raise ProductUnavailable(f"GFS network error: {exc.reason}") from exc
    except (TimeoutError, socket.timeout) as exc:
        raise ProductUnavailable(f"GFS download timeout after {timeout}s") from exc

    body = bytes(body)
    diagnostic(f"download finished httpStatus={status} bytes={len(body)} contentType={content_type} "
               f"socketTimeoutSec={timeout}")
    if len(body) < 64 or body[:4] != b"GRIB":
        preview = body[:32].hex(" ")
        raise RuntimeError(f"GFS service returned no GRIB2 data (firstBytes={preview})")

    fd, temp_name = tempfile.mkstemp(prefix=path.name + ".", suffix=".part", dir=str(cache_dir))
    try:
        with os.fdopen(fd, "wb") as output:
            output.write(body)
            output.flush()
            os.fsync(output.fileno())
        os.replace(temp_name, path)
    finally:
        if os.path.exists(temp_name):
            os.unlink(temp_name)
    diagnostic(f"cache write complete path={path} bytes={len(body)}")
    return path, "download"


def finite(value: Any) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def value_at(values: list[float], ni: int, nj: int, x: int, y: int,
             j_consecutive: bool, alternate_rows: bool) -> float:
    if j_consecutive:
        index = x * nj + y
    else:
        actual_x = ni - 1 - x if alternate_rows and y % 2 else x
        index = y * ni + actual_x
    return float(values[index])


def bool_key(handle: Any, key: str, codes: Any) -> bool:
    try:
        return bool(codes.codes_get(handle, key))
    except Exception:
        return False


def bilinear(handle: Any, latitude: float, longitude: float, codes: Any) -> float | None:
    try:
        ni = int(codes.codes_get(handle, "Ni"))
        nj = int(codes.codes_get(handle, "Nj"))
        lat_first = float(codes.codes_get(handle, "latitudeOfFirstGridPointInDegrees"))
        lat_last = float(codes.codes_get(handle, "latitudeOfLastGridPointInDegrees"))
        lon_first = float(codes.codes_get(handle, "longitudeOfFirstGridPointInDegrees"))
        lon_last = float(codes.codes_get(handle, "longitudeOfLastGridPointInDegrees"))
        values = [float(item) for item in codes.codes_get_array(handle, "values")]
        if ni < 2 or nj < 2 or len(values) < ni * nj:
            return None
        fx = (longitude - lon_first) / (lon_last - lon_first) * (ni - 1)
        fy = (latitude - lat_first) / (lat_last - lat_first) * (nj - 1)
        if not (math.isfinite(fx) and math.isfinite(fy) and 0 <= fx <= ni - 1 and 0 <= fy <= nj - 1):
            return None
        ix = min(max(int(math.floor(fx)), 0), ni - 2)
        iy = min(max(int(math.floor(fy)), 0), nj - 2)
        dx = fx - ix
        dy = fy - iy
        j_consecutive = bool_key(handle, "jPointsAreConsecutive", codes)
        alternate_rows = bool_key(handle, "alternativeRowScanning", codes)
        corners = (
            value_at(values, ni, nj, ix, iy, j_consecutive, alternate_rows),
            value_at(values, ni, nj, ix + 1, iy, j_consecutive, alternate_rows),
            value_at(values, ni, nj, ix, iy + 1, j_consecutive, alternate_rows),
            value_at(values, ni, nj, ix + 1, iy + 1, j_consecutive, alternate_rows),
        )
        if not all(finite(item) for item in corners):
            return None
        return ((1 - dx) * (1 - dy) * corners[0] + dx * (1 - dy) * corners[1] +
                (1 - dx) * dy * corners[2] + dx * dy * corners[3])
    except Exception:
        return None


def decode_profile(path: Path, address: dict[str, Any], request: dict[str, Any], codes: Any) -> dict[str, Any]:

    parts: dict[int, dict[str, float]] = {}
    with path.open("rb") as source:
        while True:
            handle = codes.codes_grib_new_from_file(source)
            if handle is None:
                break
            try:
                short_name = str(codes.codes_get(handle, "shortName"))
                level_type = str(codes.codes_get(handle, "typeOfLevel"))
                level = int(codes.codes_get(handle, "level"))
                if level_type == "isobaricInhPa" and level > 0 and short_name in {"t", "q", "gh"}:
                    value = bilinear(handle, float(request["latitudeDeg"]),
                                     float(request["longitudeDeg"]), codes)
                    if value is not None:
                        parts.setdefault(level, {})[short_name] = value
            finally:
                codes.codes_release(handle)

    levels = []
    for pressure in sorted(parts):
        item = parts[pressure]
        if not all(key in item for key in ("t", "q", "gh")):
            continue
        if item["t"] <= 100 or item["q"] < 0 or not all(finite(item[key]) for key in ("t", "q", "gh")):
            continue
        levels.append({
            "pressurePa": pressure * 100.0,
            "temperatureK": item["t"],
            "specificHumidity": item["q"],
            "geopotentialHeightM": item["gh"],
        })
    if len(levels) < 2:
        raise RuntimeError("GFS GRIB2 lacks a complete TMP/SPFH/HGT pressure profile at the requested point")
    diagnostic(f"GRIB2 decoded path={path} levels={len(levels)}")
    return {
        "validTime": address["valid"].isoformat().replace("+00:00", "Z"),
        "productTime": address["cycle"].isoformat().replace("+00:00", "Z"),
        "cycle": address["cycleText"],
        "forecastLeadSec": address["leadHour"] * 3600,
        "latitudeDeg": float(request["latitudeDeg"]),
        "longitudeDeg": float(request["longitudeDeg"]),
        "spatialInterpolated": True,
        "levels": levels,
    }


def process(request: dict[str, Any]) -> dict[str, Any]:
    requested = parse_utc(str(request["requestedUtc"]))
    request = dict(request)
    request["latitudeDeg"] = float(request["latitudeDeg"])
    request["longitudeDeg"] = float(request["longitudeDeg"])
    # Fail before any network or cache write if the isolated GRIB2 decoder was
    # not initialized. This prevents accumulating unusable cache files.
    codes = require_eccodes()
    nominal = product_address(requested)
    attempts: list[dict[str, Any]] = []
    # A newly completed GFS cycle may not be published when the real-time
    # epoch arrives. Older cycles are valid forecasts for the same epoch.
    addresses = [
        (nominal, "nominal"),
        (product_address(requested, nominal["cycle"] - dt.timedelta(hours=6)), "previous_cycle"),
        (product_address(requested, nominal["cycle"] - dt.timedelta(hours=12)), "previous_cycle_2"),
    ]
    for address, selection in addresses:
        try:
            path, source = download_to_cache(request, address)
            profile = decode_profile(path, address, request, codes)
            fallback = selection != "nominal"
            diagnostic(f"product selected cycle={address['cycleText']} leadHour={address['leadHour']} "
                       f"selection={selection} fallback={fallback}")
            return {
                "ok": True,
                "source": source,
                "selection": selection,
                "fallbackUsed": fallback,
                "cachePath": str(path),
                "productCycle": address["cycleText"],
                "forecastLeadSec": address["leadHour"] * 3600,
                "productFilename": address["filename"],
                "attempts": attempts,
                "profile": profile,
            }
        except (ProductNotFound, ProductUnavailable) as exc:
            attempts.append({
                "selection": selection,
                "cycle": address["cycleText"],
                "leadHour": address["leadHour"],
                "error": str(exc),
            })
            diagnostic(f"product unavailable, trying fallback cycle={address['cycleText']} "
                       f"leadHour={address['leadHour']} reason={exc}")
    attempted = ", ".join(
        f"{item['cycle']}+f{item['leadHour']:03d}" for item in attempts
    )
    raise RuntimeError(f"GFS product unavailable after fallback attempts: {attempted}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--request", default="-", help="JSON request path, or - for stdin")
    args = parser.parse_args()
    try:
        input_stream = sys.stdin if args.request == "-" else open(args.request, "r", encoding="utf-8")
        with input_stream:
            request = json.load(input_stream)
        response = process(request)
    except Exception as exc:
        diagnostic(f"failed: {exc}")
        response = {"ok": False, "error": str(exc)}
    print(json.dumps(response, ensure_ascii=True, separators=(",", ":")), flush=True)
    return 0 if response.get("ok") else 1


if __name__ == "__main__":
    raise SystemExit(main())
