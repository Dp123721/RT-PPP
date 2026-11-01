"""Loads and summarizes PPP prefit/postfit residual telemetry."""

from __future__ import annotations

import csv
import math
from datetime import datetime
from pathlib import Path
from typing import Any

from ..core.time import datetime_to_unix_ns
from ..models import MissingReason


class ResidualError(ValueError):
    pass


def _float(text: str, context: str) -> float:
    try:
        value = float(text)
    except ValueError as exc:
        raise ResidualError(f"{context}: invalid number {text!r}") from exc
    if not math.isfinite(value):
        raise ResidualError(f"{context}: non-finite number")
    return value


def _bool(text: str, context: str) -> bool:
    normalized = text.strip().lower()
    if normalized not in {"true", "false"}:
        raise ResidualError(f"{context}: invalid boolean {text!r}")
    return normalized == "true"


def read_residuals(path_value: Path, source_id: str | None = None) -> list[dict[str, Any]]:
    paths = sorted(path_value.glob("ppp_residual_*.csv")) if path_value.is_dir() else [path_value]
    if not paths or any(not path.is_file() for path in paths):
        raise ResidualError(f"PPP residual input does not exist: {path_value}")
    required = {
        "source_id", "epoch_utc", "satellite", "signal", "observation_type",
        "prefit_m", "postfit_m", "sigma_m", "elevation_deg", "adopted", "reason",
    }
    allowed_reasons = {reason.value for reason in MissingReason}
    result: list[dict[str, Any]] = []
    seen: set[tuple[str, int, str, str, str]] = set()
    for path in paths:
        with path.open("r", encoding="utf-8-sig", newline="") as stream:
            reader = csv.DictReader(stream)
            missing = required - set(reader.fieldnames or [])
            if missing:
                raise ResidualError(f"{path}: missing columns {sorted(missing)}")
            for line, raw in enumerate(reader, start=2):
                context = f"{path}:{line}"
                row = {key: (value or "").strip() for key, value in raw.items()}
                if source_id and row["source_id"] != source_id:
                    continue
                if not row["satellite"] or not row["signal"]:
                    raise ResidualError(f"{context}: satellite and signal are required")
                if row["observation_type"] not in {"code", "phase", "iono"}:
                    raise ResidualError(f"{context}: invalid observation_type")
                try:
                    epoch = datetime.fromisoformat(row["epoch_utc"].replace("Z", "+00:00"))
                except ValueError as exc:
                    raise ResidualError(f"{context}: invalid epoch_utc") from exc
                if epoch.tzinfo is None:
                    raise ResidualError(f"{context}: epoch_utc requires timezone")
                adopted = _bool(row["adopted"], context)
                reason = row["reason"]
                if reason and reason not in allowed_reasons:
                    raise ResidualError(f"{context}: invalid reason {reason!r}")
                if adopted and reason:
                    raise ResidualError(f"{context}: adopted residual cannot have a rejection reason")
                if not adopted and not reason:
                    reason = MissingReason.UNKNOWN.value
                sigma = _float(row["sigma_m"], context)
                if sigma <= 0.0:
                    raise ResidualError(f"{context}: sigma_m must be positive")
                epoch_ns = datetime_to_unix_ns(epoch)
                key = (
                    row["source_id"], epoch_ns, row["satellite"], row["signal"],
                    row["observation_type"],
                )
                if key in seen:
                    raise ResidualError(f"{context}: duplicate final-iteration residual {key}")
                seen.add(key)
                result.append({
                    **row,
                    "epoch_ns": epoch_ns,
                    "prefit_m": _float(row["prefit_m"], context),
                    "postfit_m": _float(row["postfit_m"], context),
                    "sigma_m": sigma,
                    "elevation_deg": _float(row["elevation_deg"], context),
                    "adopted": adopted,
                    "reason": reason,
                })
    return result
