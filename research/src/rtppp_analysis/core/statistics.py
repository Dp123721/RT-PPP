"""Implements shared robust statistics and uncertainty metrics."""

from __future__ import annotations

import math
from typing import Iterable

import numpy as np


def finite_array(values: Iterable[float]) -> np.ndarray:
    array = np.asarray(list(values), dtype=float)
    return array[np.isfinite(array)]


def describe(values: Iterable[float]) -> dict[str, float | int]:
    materialized = list(values)
    total_count = len(materialized)
    array = finite_array(materialized)
    count = int(array.size)
    coverage_rate = count / total_count if total_count else float("nan")
    coverage = {
        "count": count,
        "total_count": total_count,
        "missing_count": total_count - count,
        "coverage_rate": coverage_rate,
    }
    if array.size == 0:
        return {
            **coverage,
            "mean": float("nan"),
            "std": float("nan"),
            "rms": float("nan"),
            "median": float("nan"),
            "p68_abs": float("nan"),
            "p95_abs": float("nan"),
            "max_abs": float("nan"),
        }
    return {
        **coverage,
        "mean": float(np.mean(array)),
        "std": float(np.std(array)),
        "rms": float(math.sqrt(float(np.mean(array * array)))),
        "median": float(np.median(array)),
        "p68_abs": float(np.percentile(np.abs(array), 68.0)),
        "p95_abs": float(np.percentile(np.abs(array), 95.0)),
        "max_abs": float(np.max(np.abs(array))),
    }


def sustained_convergence_seconds(
    epoch_ns: np.ndarray,
    horizontal_m: np.ndarray,
    vertical_m: np.ndarray,
    horizontal_threshold_m: float,
    vertical_threshold_m: float,
    sustain_seconds: float,
    max_gap_seconds: float | None = None,
) -> float:
    if epoch_ns.size == 0:
        return float("nan")
    if epoch_ns.ndim != 1 or horizontal_m.shape != epoch_ns.shape or vertical_m.shape != epoch_ns.shape:
        raise ValueError("Convergence inputs must be one-dimensional arrays of equal length")
    epoch_gaps_ns = np.diff(epoch_ns)
    if np.any(epoch_gaps_ns <= 0):
        raise ValueError("Convergence epochs must be strictly increasing")
    max_gap_ns = None
    if max_gap_seconds is not None and np.isfinite(max_gap_seconds):
        if max_gap_seconds <= 0.0:
            raise ValueError("Maximum convergence gap must be positive")
        max_gap_ns = int(round(max_gap_seconds * 1e9))
    good = (
        np.isfinite(horizontal_m)
        & np.isfinite(vertical_m)
        & (horizontal_m <= horizontal_threshold_m)
        & (vertical_m <= vertical_threshold_m)
    )
    for start in np.flatnonzero(good):
        target = epoch_ns[start] + int(round(sustain_seconds * 1e9))
        end = int(np.searchsorted(epoch_ns, target, side="left"))
        if end >= epoch_ns.size:
            continue
        has_large_gap = max_gap_ns is not None and np.any(epoch_gaps_ns[start:end] > max_gap_ns)
        if not has_large_gap and np.all(good[start : end + 1]):
            return (epoch_ns[start] - epoch_ns[0]) / 1e9
    return float("nan")
