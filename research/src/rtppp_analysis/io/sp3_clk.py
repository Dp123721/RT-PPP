"""Parses SP3 orbit and clock products into analysis records."""

from __future__ import annotations

import re
from pathlib import Path

import numpy as np

from ..core.time import calendar_epoch_to_utc, datetime_to_unix_ns
from ..models import ClockPoint, OrbitPoint, Product


class ProductParseError(ValueError):
    pass


_SATELLITE_PATTERN = re.compile(r"^[A-Z][0-9]{2}$")
_SP3_MISSING_LIMIT = 999999.0


def _calendar_fields(parts: list[str], start: int = 0) -> tuple[int, int, int, int, int, float]:
    return (
        int(parts[start]),
        int(parts[start + 1]),
        int(parts[start + 2]),
        int(parts[start + 3]),
        int(parts[start + 4]),
        float(parts[start + 5]),
    )


def _satellite_id(value: str, path: Path, line_number: int, record_type: str) -> str:
    satellite = value.strip().upper()
    if not _SATELLITE_PATTERN.fullmatch(satellite):
        raise ProductParseError(
            f"Invalid {record_type} satellite identifier at {path}:{line_number}"
        )
    return satellite


def read_sp3(path: Path, time_scale: str) -> dict[str, list[OrbitPoint]]:
    if not path.is_file():
        raise FileNotFoundError(path)
    current_epoch_ns: int | None = None
    indexed: dict[tuple[int, str], OrbitPoint] = {}
    missing_position_keys: set[tuple[int, str]] = set()
    seen_records: set[tuple[int, str, str]] = set()
    saw_epoch = False
    with path.open("r", encoding="utf-8", errors="replace") as stream:
        for line_number, line in enumerate(stream, start=1):
            if line.startswith("*"):
                parts = line[1:].split()
                if len(parts) < 6:
                    raise ProductParseError(f"Invalid SP3 epoch at {path}:{line_number}")
                try:
                    utc = calendar_epoch_to_utc(
                        *_calendar_fields(parts), time_scale=time_scale
                    )
                except (ValueError, IndexError) as exc:
                    raise ProductParseError(
                        f"Invalid SP3 epoch at {path}:{line_number}"
                    ) from exc
                current_epoch_ns = datetime_to_unix_ns(utc)
                saw_epoch = True
                continue
            if not line or line[0] not in {"P", "V"}:
                continue
            record_type = line[0]
            if current_epoch_ns is None:
                raise ProductParseError(
                    f"SP3 {record_type} record appears before an epoch at {path}:{line_number}"
                )
            if len(line) < 4:
                raise ProductParseError(f"Invalid SP3 record at {path}:{line_number}")
            satellite = _satellite_id(line[1:4], path, line_number, f"SP3 {record_type}")
            fields = line[4:].split()
            if len(fields) < 4:
                raise ProductParseError(f"Invalid SP3 record at {path}:{line_number}")
            try:
                values = [float(value) for value in fields[:4]]
            except ValueError as exc:
                raise ProductParseError(f"Invalid SP3 record at {path}:{line_number}") from exc
            if not np.all(np.isfinite(values)):
                raise ProductParseError(
                    f"Non-finite SP3 {record_type} record at {path}:{line_number}"
                )
            key = (current_epoch_ns, satellite)
            record_key = (current_epoch_ns, satellite, record_type)
            if record_key in seen_records:
                raise ProductParseError(
                    f"Duplicate SP3 {record_type} record at {path}:{line_number}"
                )
            seen_records.add(record_key)
            if record_type == "P":
                position_km = np.asarray(values[:3], dtype=float)
                if np.all(position_km == 0.0) or np.any(
                    np.abs(position_km) >= _SP3_MISSING_LIMIT
                ):
                    missing_position_keys.add(key)
                    continue
                position = position_km * 1000.0
                clock_s = values[3] * 1e-6 if abs(values[3]) < 999999.0 else float("nan")
                indexed[key] = OrbitPoint(current_epoch_ns, satellite, position, clock_s)
            elif key in missing_position_keys:
                continue
            elif key not in indexed:
                raise ProductParseError(
                    f"SP3 V record has no preceding P record at {path}:{line_number}"
                )
            else:
                velocity_dmps = np.asarray(values[:3], dtype=float)
                if np.all(velocity_dmps == 0.0) or np.any(
                    np.abs(velocity_dmps) >= _SP3_MISSING_LIMIT
                ):
                    continue
                indexed[key].velocity_mps = velocity_dmps / 10.0

    if not saw_epoch:
        raise ProductParseError(f"SP3 file contains no epoch records: {path}")

    grouped: dict[str, list[OrbitPoint]] = {}
    for point in indexed.values():
        grouped.setdefault(point.satellite, []).append(point)
    for points in grouped.values():
        points.sort(key=lambda point: point.epoch_ns)
    return grouped


def read_clk(path: Path, time_scale: str) -> dict[str, list[ClockPoint]]:
    if not path.is_file():
        raise FileNotFoundError(path)
    grouped: dict[str, list[ClockPoint]] = {}
    seen_records: set[tuple[int, str]] = set()
    found_header_end = False
    with path.open("r", encoding="utf-8", errors="replace") as stream:
        in_header = True
        for line_number, line in enumerate(stream, start=1):
            if in_header:
                if "END OF HEADER" in line:
                    in_header = False
                    found_header_end = True
                continue
            parts = line.split()
            if not parts or parts[0] != "AS":
                continue
            if len(parts) < 9:
                raise ProductParseError(f"Invalid CLK AS record at {path}:{line_number}")
            satellite = _satellite_id(parts[1], path, line_number, "CLK AS")
            try:
                utc = calendar_epoch_to_utc(*_calendar_fields(parts, 2), time_scale=time_scale)
                remaining = parts[8:]
                if len(remaining) >= 2 and remaining[0].isdigit():
                    declared_count = int(remaining[0])
                    if declared_count < 1 or len(remaining) < declared_count + 1:
                        raise ValueError("CLK value count does not match the record")
                    clock_s = float(remaining[1])
                else:
                    clock_s = float(remaining[0])
            except (ValueError, IndexError) as exc:
                raise ProductParseError(f"Invalid CLK record at {path}:{line_number}") from exc
            if not np.isfinite(clock_s):
                raise ProductParseError(f"Non-finite CLK record at {path}:{line_number}")
            point = ClockPoint(datetime_to_unix_ns(utc), satellite, clock_s)
            key = (point.epoch_ns, satellite)
            if key in seen_records:
                raise ProductParseError(f"Duplicate CLK AS record at {path}:{line_number}")
            seen_records.add(key)
            grouped.setdefault(satellite, []).append(point)
    if not found_header_end:
        raise ProductParseError(f"CLK header is incomplete: {path}")
    for points in grouped.values():
        points.sort(key=lambda point: point.epoch_ns)
    return grouped


def load_product(product_config: dict, base_path: Path) -> Product:
    product_id = str(product_config.get("id", "")).strip()
    if not product_id:
        raise ValueError("Product id is required")
    time_scale = str(product_config.get("time_scale", "")).strip().upper()
    if time_scale not in {"GPS", "UTC"}:
        raise ValueError(f"Product {product_id} must declare time_scale as GPS or UTC")
    orbit_reference = str(product_config.get("orbit_reference", "UNKNOWN")).strip().upper()
    if orbit_reference not in {"APC", "COM", "UNKNOWN"}:
        raise ValueError(f"Unsupported orbit_reference for {product_id}: {orbit_reference}")
    reference_frame = str(product_config.get("reference_frame", "")).strip().upper()
    if not reference_frame:
        raise ValueError(f"Product {product_id} requires reference_frame")

    sp3_value = product_config.get("sp3")
    if not isinstance(sp3_value, str) or not sp3_value.strip():
        raise ValueError(f"Product {product_id} requires an SP3 path")
    sp3_path = Path(sp3_value).expanduser()
    if not sp3_path.is_absolute():
        sp3_path = base_path / sp3_path
    sp3_path = sp3_path.resolve()
    orbits = read_sp3(sp3_path, time_scale)

    input_paths = [str(sp3_path)]
    clk_value = product_config.get("clk")
    if isinstance(clk_value, str) and clk_value.strip():
        clk_path = Path(clk_value).expanduser()
        if not clk_path.is_absolute():
            clk_path = base_path / clk_path
        clk_path = clk_path.resolve()
        clocks = read_clk(clk_path, time_scale)
        input_paths.append(str(clk_path))
    else:
        clocks = {
            satellite: [
                ClockPoint(point.epoch_ns, satellite, point.clock_s)
                for point in points
                if np.isfinite(point.clock_s)
            ]
            for satellite, points in orbits.items()
        }
    return Product(
        product_id,
        time_scale,
        orbit_reference,
        reference_frame,
        orbits,
        clocks,
        input_paths,
    )
