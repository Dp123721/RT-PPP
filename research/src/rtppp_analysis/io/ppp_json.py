"""Loads compact PPP JSON files into typed solution records."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import numpy as np

from ..config import ContractError
from ..contracts import validate_compact
from ..core.time import datetime_to_unix_ns
from ..models import PppEpoch


class PppJsonError(ValueError):
    pass


def _numeric_array(value: Any, length: int) -> np.ndarray:
    if not isinstance(value, list) or len(value) < length:
        return np.full(length, np.nan)
    return np.asarray([float(item) if item is not None else np.nan for item in value[:length]])


def _iso_epoch_ns(value: str) -> int:
    from datetime import datetime

    normalized = value.replace("Z", "+00:00")
    return datetime_to_unix_ns(datetime.fromisoformat(normalized))


def read_ppp_json(path: Path) -> tuple[dict[str, Any], list[PppEpoch]]:
    if not path.is_file():
        raise FileNotFoundError(path)
    if path.suffix == ".part":
        raise PppJsonError("Runtime .part files are not finalized JSON; close or rotate the writer first")
    try:
        root = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise PppJsonError(f"Invalid PPP JSON {path}: {exc}") from exc
    try:
        validate_compact(root)
    except ContractError as exc:
        raise PppJsonError(f"PPP JSON contract violation: {exc}") from exc
    meta = root.get("meta")
    solutions = root.get("solutions")
    if not isinstance(meta, dict) or not isinstance(solutions, list):
        raise PppJsonError("PPP JSON requires meta and solutions")
    if meta.get("schema") != "rtppp.compact" or int(meta.get("version", 0)) != 1:
        raise PppJsonError("Unsupported PPP JSON schema/version")
    time_base = meta.get("timeBase", {})
    base_epoch_ns = _iso_epoch_ns(str(time_base.get("epoch", "")))
    algorithms = meta.get("algorithms", [])
    statuses = meta.get("arStatus", [])

    epochs: list[PppEpoch] = []
    seen_epochs: set[int] = set()
    for row_number, row in enumerate(solutions):
        if not isinstance(row, list) or len(row) < 11:
            raise PppJsonError(f"Invalid solution row {row_number}")
        algorithm_index = int(row[1])
        status_index = int(row[2])
        if not 0 <= algorithm_index < len(algorithms):
            raise PppJsonError(f"Invalid algorithm index in solution row {row_number}")
        if not 0 <= status_index < len(statuses):
            raise PppJsonError(f"Invalid AR status index in solution row {row_number}")
        algorithm = algorithms[algorithm_index]
        status = statuses[status_index]
        tropo = _numeric_array(row[6], 3)
        satellite_summary = _numeric_array(row[7], 4)
        covariance_values = _numeric_array(row[5], 6)
        covariance = np.array(
            [
                [covariance_values[0], covariance_values[1], covariance_values[2]],
                [covariance_values[1], covariance_values[3], covariance_values[4]],
                [covariance_values[2], covariance_values[4], covariance_values[5]],
            ]
        )
        epoch_ns = base_epoch_ns + int(round(float(row[0]) * 1_000_000))
        if epoch_ns in seen_epochs:
            raise PppJsonError(f"Duplicate PPP epoch in solution row {row_number}")
        seen_epochs.add(epoch_ns)
        epochs.append(
            PppEpoch(
                epoch_ns=epoch_ns,
                algorithm=algorithm,
                ar_status=status,
                ecef_m=_numeric_array(row[3], 3),
                stored_enu_m=_numeric_array(row[4], 3),
                covariance_ecef_m2=covariance,
                ztd_m=tropo[0],
                wet_residual_m=tropo[1],
                ztd_std_m=tropo[2],
                used_count=int(satellite_summary[0]) if np.isfinite(satellite_summary[0]) else 0,
                eligible_count=int(satellite_summary[1]) if np.isfinite(satellite_summary[1]) else 0,
                fixed_count=int(satellite_summary[2]) if np.isfinite(satellite_summary[2]) else 0,
                hdop=satellite_summary[3],
                satellite_sets=row[8] if isinstance(row[8], list) else [],
                ar_data=row[9],
                flags=int(row[10]),
            )
        )
    epochs.sort(key=lambda epoch: epoch.epoch_ns)
    return meta, epochs
