"""Parses receiver telemetry and derives per-epoch signal availability."""

from __future__ import annotations

import csv
import math
from collections import defaultdict
from datetime import datetime
from pathlib import Path
from typing import Any, Iterable

import numpy as np

from ..core.statistics import describe


class TelemetryError(ValueError):
    pass


MISSING_REASONS = {
    "not_transmitted", "not_received", "stream_interruption", "stale",
    "iod_mismatch", "rejected", "invalid", "unknown",
}


def _utc_ns(text: str, path: Path, row_number: int) -> int:
    try:
        value = datetime.fromisoformat(text.replace("Z", "+00:00"))
    except ValueError as exc:
        raise TelemetryError(f"{path}:{row_number}: invalid UTC timestamp {text!r}") from exc
    if value.tzinfo is None:
        raise TelemetryError(f"{path}:{row_number}: UTC timestamp requires a timezone")
    return int(round(value.timestamp() * 1_000_000_000))


def _float(text: str) -> float:
    if not text:
        return float("nan")
    try:
        value = float(text)
    except ValueError as exc:
        raise TelemetryError(f"invalid numeric telemetry value {text!r}") from exc
    return value if math.isfinite(value) else float("nan")


def _bool(text: str) -> bool:
    normalized = text.strip().lower()
    if normalized not in {"true", "false"}:
        raise TelemetryError(f"invalid boolean telemetry value {text!r}")
    return normalized == "true"


def _read_family(directory: Path, family: str, required: set[str]) -> list[dict[str, str]]:
    rows: list[dict[str, str]] = []
    for path in sorted(directory.glob(f"{family}_*.csv")):
        with path.open("r", encoding="utf-8-sig", newline="") as stream:
            reader = csv.DictReader(stream)
            fields = set(reader.fieldnames or [])
            missing = required - fields
            if missing:
                raise TelemetryError(f"{path}: missing columns {sorted(missing)}")
            for row_number, row in enumerate(reader, start=2):
                normalized = {key: (value or "").strip() for key, value in row.items()}
                normalized["_path"] = str(path)
                normalized["_row"] = str(row_number)
                rows.append(normalized)
    return rows


def read_telemetry(directory: Path, source_id: str | None = None) -> dict[str, list[dict[str, Any]]]:
    if not directory.is_dir():
        raise TelemetryError(f"Telemetry directory does not exist: {directory}")
    msm_raw = _read_family(
        directory,
        "msm",
        {"source_id", "received_utc", "epoch_utc", "station_id", "system", "satellite",
         "signal", "has_lock_time", "lock_time_seconds", "has_half_cycle",
         "half_cycle_ambiguity", "has_cnr", "cnr_dbhz", "elevation_deg"},
    )
    ssr_raw = _read_family(
        directory,
        "ssr",
        {"source_id", "received_utc", "epoch_utc", "message_type", "kind", "system",
         "satellite", "age_seconds", "adopted", "reason"},
    )
    reception_raw = _read_family(
        directory,
        "reception",
        {"source_id", "received_utc", "message_family", "message_type", "system",
         "status", "reason", "detail"},
    )
    if source_id:
        msm_raw = [row for row in msm_raw if row["source_id"] == source_id]
        ssr_raw = [row for row in ssr_raw if row["source_id"] == source_id]
        reception_raw = [row for row in reception_raw if row["source_id"] == source_id]

    msm: list[dict[str, Any]] = []
    for row in msm_raw:
        path = Path(row["_path"])
        number = int(row["_row"])
        if not row["epoch_utc"] or not row["satellite"] or not row["signal"]:
            raise TelemetryError(f"{path}:{number}: MSM epoch, satellite, and signal are required")
        msm.append(
            {
                **{key: value for key, value in row.items() if not key.startswith("_")},
                "epoch_ns": _utc_ns(row["epoch_utc"], path, number),
                "received_ns": _utc_ns(row["received_utc"], path, number),
                "has_lock_time": _bool(row["has_lock_time"]),
                "lock_time_seconds": _float(row["lock_time_seconds"]),
                "has_half_cycle": _bool(row["has_half_cycle"]),
                "half_cycle_ambiguity": int(row["half_cycle_ambiguity"]) if row["half_cycle_ambiguity"] else None,
                "has_cnr": _bool(row["has_cnr"]),
                "cnr_dbhz": _float(row["cnr_dbhz"]),
                "elevation_deg": _float(row["elevation_deg"]),
            }
        )
    ssr: list[dict[str, Any]] = []
    for row in ssr_raw:
        path = Path(row["_path"])
        number = int(row["_row"])
        if row["reason"] and row["reason"] not in MISSING_REASONS:
            raise TelemetryError(f"{path}:{number}: invalid SSR reason {row['reason']!r}")
        ssr.append(
            {
                **{key: value for key, value in row.items() if not key.startswith("_")},
                "epoch_ns": _utc_ns(row["epoch_utc"], path, number),
                "received_ns": _utc_ns(row["received_utc"], path, number),
                "age_seconds": _float(row["age_seconds"]),
                "adopted": _bool(row["adopted"]),
            }
        )
    reception: list[dict[str, Any]] = []
    for row in reception_raw:
        path = Path(row["_path"])
        number = int(row["_row"])
        if row["reason"] and row["reason"] not in MISSING_REASONS:
            raise TelemetryError(f"{path}:{number}: invalid reception reason {row['reason']!r}")
        reception.append(
            {
                **{key: value for key, value in row.items() if not key.startswith("_")},
                "received_ns": _utc_ns(row["received_utc"], path, number),
            }
        )
    return {"msm": msm, "ssr": ssr, "reception": reception}


def analyze_tracking(
    msm_rows: Iterable[dict[str, Any]], expected_interval_seconds: float, gap_factor: float
) -> tuple[list[dict[str, Any]], list[dict[str, Any]], list[dict[str, Any]]]:
    groups: dict[tuple[str, str, str], list[dict[str, Any]]] = defaultdict(list)
    for row in msm_rows:
        groups[(row["source_id"], row["satellite"], row["signal"])].append(row)
    arcs: list[dict[str, Any]] = []
    slips: list[dict[str, Any]] = []
    half_events: list[dict[str, Any]] = []
    for (source, satellite, signal), values in sorted(groups.items()):
        by_epoch: dict[int, dict[str, Any]] = {}
        for row in values:
            old = by_epoch.get(row["epoch_ns"])
            if old is None or sum(bool(row.get(key)) for key in ("has_lock_time", "has_half_cycle", "has_cnr")) > sum(
                bool(old.get(key)) for key in ("has_lock_time", "has_half_cycle", "has_cnr")
            ):
                by_epoch[row["epoch_ns"]] = row
        ordered = [by_epoch[key] for key in sorted(by_epoch)]
        if not ordered:
            continue
        start = 0
        arc_slips = 0
        for index in range(1, len(ordered)):
            previous = ordered[index - 1]
            current = ordered[index]
            gap_seconds = (current["epoch_ns"] - previous["epoch_ns"]) / 1e9
            lock_reset = (
                previous["has_lock_time"] and current["has_lock_time"]
                and np.isfinite(previous["lock_time_seconds"])
                and np.isfinite(current["lock_time_seconds"])
                and current["lock_time_seconds"] + max(gap_seconds, 0.0) * 0.25 < previous["lock_time_seconds"]
            )
            half_change = (
                previous["has_half_cycle"] and current["has_half_cycle"]
                and previous["half_cycle_ambiguity"] != current["half_cycle_ambiguity"]
            )
            if half_change:
                half_events.append(
                    {"source_id": source, "satellite": satellite, "signal": signal,
                     "epoch_ns": current["epoch_ns"], "epoch_utc": current["epoch_utc"],
                     "previous_half_cycle": previous["half_cycle_ambiguity"],
                     "half_cycle": current["half_cycle_ambiguity"]}
                )
            if lock_reset or half_change:
                evidence = "lock_time_reset" if lock_reset else "half_cycle_change"
                slips.append(
                    {"source_id": source, "satellite": satellite, "signal": signal,
                     "epoch_ns": current["epoch_ns"], "epoch_utc": current["epoch_utc"],
                     "evidence": evidence, "previous_lock_seconds": previous["lock_time_seconds"],
                     "lock_seconds": current["lock_time_seconds"]}
                )
                arc_slips += 1
            large_gap = expected_interval_seconds > 0 and gap_seconds > expected_interval_seconds * gap_factor
            if lock_reset or half_change or large_gap:
                end = ordered[index - 1]
                arcs.append(_arc_row(source, satellite, signal, ordered[start : index], arc_slips,
                                     evidence if (lock_reset or half_change) else "unknown"))
                start = index
                arc_slips = 0
        arcs.append(_arc_row(source, satellite, signal, ordered[start:], arc_slips, "end_of_data"))
    return arcs, slips, half_events


def _arc_row(
    source: str, satellite: str, signal: str, rows: list[dict[str, Any]], slip_count: int,
    end_reason: str,
) -> dict[str, Any]:
    first, last = rows[0], rows[-1]
    return {
        "source_id": source,
        "satellite": satellite,
        "signal": signal,
        "start_epoch_ns": first["epoch_ns"],
        "start_epoch_utc": first["epoch_utc"],
        "end_epoch_ns": last["epoch_ns"],
        "end_epoch_utc": last["epoch_utc"],
        "duration_seconds": (last["epoch_ns"] - first["epoch_ns"]) / 1e9,
        "epoch_count": len(rows),
        "cycle_slip_count": slip_count,
        "end_reason": end_reason,
    }


def elevation_snr_summary(msm_rows: Iterable[dict[str, Any]]) -> list[dict[str, Any]]:
    groups: dict[tuple[str, str, str, int], list[float]] = defaultdict(list)
    for row in msm_rows:
        elevation = row["elevation_deg"]
        cnr = row["cnr_dbhz"]
        if not row["has_cnr"] or not np.isfinite(elevation) or not np.isfinite(cnr):
            continue
        lower = min(80, max(0, int(elevation // 10) * 10))
        groups[(row["source_id"], row["satellite"], row["signal"], lower)].append(cnr)
    result: list[dict[str, Any]] = []
    for (source, satellite, signal, lower), values in sorted(groups.items()):
        result.append(
            {"source_id": source, "satellite": satellite, "signal": signal,
             "elevation_min_deg": lower, "elevation_max_deg": 90 if lower == 80 else lower + 10,
             **{f"snr_{key}": value for key, value in describe(values).items()}}
        )
    return result


def write_partitioned_parquet(
    root: Path, station: str, families: dict[str, list[dict[str, Any]]]
) -> list[Path]:
    try:
        import pandas as pd
    except ImportError as exc:
        raise RuntimeError("Parquet export requires the 'full' extra (pandas and pyarrow)") from exc
    written: list[Path] = []
    for family, rows in families.items():
        partitions: dict[tuple[str, str], list[dict[str, Any]]] = defaultdict(list)
        time_key = "epoch_utc" if family != "reception" else "received_utc"
        for row in rows:
            date = str(row[time_key])[:10]
            partitions[(date, str(row["source_id"]))].append(row)
        for (date, source), values in partitions.items():
            directory = root / f"date={date}" / f"station={station}" / f"source={source}"
            directory.mkdir(parents=True, exist_ok=True)
            path = directory / f"{family}.parquet"
            frame = pd.DataFrame(values)
            frame.to_parquet(path, compression="zstd", index=False)
            written.append(path)
    return written
