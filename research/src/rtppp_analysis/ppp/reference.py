"""Loads and interpolates external station reference coordinates."""

from __future__ import annotations

import csv
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any

import numpy as np

from ..core.time import datetime_to_unix_ns


class ReferenceError(ValueError):
    pass


def _epoch_ns(text: str, context: str) -> int:
    try:
        value = datetime.fromisoformat(text.replace("Z", "+00:00"))
    except ValueError as exc:
        raise ReferenceError(f"{context}: invalid UTC epoch {text!r}") from exc
    if value.tzinfo is None:
        raise ReferenceError(f"{context}: UTC epoch requires a timezone")
    return datetime_to_unix_ns(value)


def _vector(value: Any, name: str) -> np.ndarray:
    array = np.asarray(value, dtype=float)
    if array.shape != (3,) or not np.all(np.isfinite(array)):
        raise ReferenceError(f"{name} must contain exactly three finite values")
    return array


@dataclass(frozen=True)
class ReferenceProvider:
    mode: str
    base_ecef_m: np.ndarray
    base_epoch_ns: int | None = None
    velocity_ecef_mps: np.ndarray | None = None
    trajectory_epoch_ns: np.ndarray | None = None
    trajectory_ecef_m: np.ndarray | None = None
    source_path: Path | None = None

    def at(self, epoch_ns: int) -> np.ndarray:
        if self.mode == "trajectory":
            assert self.trajectory_epoch_ns is not None
            assert self.trajectory_ecef_m is not None
            epochs = self.trajectory_epoch_ns
            if epoch_ns < int(epochs[0]) or epoch_ns > int(epochs[-1]):
                return np.full(3, np.nan)
            exact = int(np.searchsorted(epochs, epoch_ns))
            if exact < epochs.size and int(epochs[exact]) == epoch_ns:
                return self.trajectory_ecef_m[exact].copy()
            upper = exact
            lower = upper - 1
            fraction = (epoch_ns - int(epochs[lower])) / (int(epochs[upper]) - int(epochs[lower]))
            return self.trajectory_ecef_m[lower] + fraction * (
                self.trajectory_ecef_m[upper] - self.trajectory_ecef_m[lower]
            )
        if self.mode == "velocity":
            assert self.base_epoch_ns is not None
            assert self.velocity_ecef_mps is not None
            return self.base_ecef_m + self.velocity_ecef_mps * (
                (epoch_ns - self.base_epoch_ns) / 1e9
            )
        return self.base_ecef_m.copy()


def load_reference(dataset: dict[str, Any], config_path: Path) -> ReferenceProvider:
    trajectory_value = str(dataset.get("truth_trajectory", "")).strip()
    configured = dataset.get("reference_ecef_m")
    base = _vector(configured, "reference_ecef_m") if configured is not None else np.full(3, np.nan)
    if trajectory_value:
        path = Path(trajectory_value).expanduser()
        if not path.is_absolute():
            path = config_path.parent / path
        path = path.resolve()
        if not path.is_file():
            raise FileNotFoundError(path)
        epochs: list[int] = []
        positions: list[list[float]] = []
        with path.open("r", encoding="utf-8-sig", newline="") as stream:
            reader = csv.DictReader(stream)
            required = {"epoch_utc", "x_m", "y_m", "z_m"}
            missing = required - set(reader.fieldnames or [])
            if missing:
                raise ReferenceError(f"{path}: missing columns {sorted(missing)}")
            for line, row in enumerate(reader, start=2):
                epoch = _epoch_ns(str(row["epoch_utc"]), f"{path}:{line}")
                try:
                    xyz = [float(row[key]) for key in ("x_m", "y_m", "z_m")]
                except (TypeError, ValueError) as exc:
                    raise ReferenceError(f"{path}:{line}: invalid ECEF coordinates") from exc
                if not np.all(np.isfinite(xyz)):
                    raise ReferenceError(f"{path}:{line}: non-finite ECEF coordinates")
                epochs.append(epoch)
                positions.append(xyz)
        order = np.argsort(np.asarray(epochs, dtype=np.int64))
        sorted_epochs = np.asarray(epochs, dtype=np.int64)[order]
        if sorted_epochs.size < 2:
            raise ReferenceError("Dynamic truth trajectory requires at least two epochs")
        if np.any(np.diff(sorted_epochs) <= 0):
            raise ReferenceError("Dynamic truth trajectory epochs must be unique and increasing")
        sorted_positions = np.asarray(positions, dtype=float)[order]
        return ReferenceProvider(
            "trajectory", sorted_positions[0], trajectory_epoch_ns=sorted_epochs,
            trajectory_ecef_m=sorted_positions, source_path=path,
        )
    velocity_value = dataset.get("reference_velocity_ecef_mps")
    if velocity_value is not None:
        velocity = _vector(velocity_value, "reference_velocity_ecef_mps")
        epoch_text = str(dataset.get("reference_epoch_utc", "")).strip()
        if not epoch_text:
            raise ReferenceError("reference_epoch_utc is required with reference velocity")
        return ReferenceProvider(
            "velocity", base, _epoch_ns(epoch_text, "reference_epoch_utc"), velocity
        )
    return ReferenceProvider("static", base)
