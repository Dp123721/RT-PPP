"""Computes PPP convergence, availability, and ambiguity-resolution metrics."""

from __future__ import annotations

import math
from typing import Any, Iterable

import numpy as np

from ..core.statistics import sustained_convergence_seconds


def paired_sign_test_pvalue(values: Iterable[float]) -> float:
    array = np.asarray(list(values), dtype=float)
    array = array[np.isfinite(array) & (array != 0.0)]
    n = int(array.size)
    if n == 0:
        return float("nan")
    positive = int(np.sum(array > 0.0))
    tail = min(positive, n - positive)
    probability = sum(math.comb(n, index) for index in range(tail + 1)) / (2.0**n)
    return min(1.0, 2.0 * probability)


def bootstrap_mean_ci(
    values: Iterable[float], samples: int, confidence: float, seed: int
) -> tuple[float, float]:
    array = np.asarray(list(values), dtype=float)
    array = array[np.isfinite(array)]
    if array.size == 0 or samples <= 0:
        return float("nan"), float("nan")
    rng = np.random.default_rng(seed)
    indices = rng.integers(0, array.size, size=(samples, array.size))
    means = np.mean(array[indices], axis=1)
    tail = (1.0 - confidence) / 2.0
    return float(np.quantile(means, tail)), float(np.quantile(means, 1.0 - tail))


def detect_robust_jumps(
    rows: list[dict[str, Any]], mad_factor: float, minimum_jump_m: float
) -> tuple[list[dict[str, Any]], float]:
    if len(rows) < 2:
        return [], minimum_jump_m
    steps = np.asarray([
        np.linalg.norm([
            rows[index]["north_m"] - rows[index - 1]["north_m"],
            rows[index]["east_m"] - rows[index - 1]["east_m"],
            rows[index]["up_m"] - rows[index - 1]["up_m"],
        ])
        for index in range(1, len(rows))
    ], dtype=float)
    finite = steps[np.isfinite(steps)]
    if finite.size:
        median = float(np.median(finite))
        mad = float(np.median(np.abs(finite - median)))
        threshold = max(minimum_jump_m, median + mad_factor * 1.4826 * mad)
    else:
        threshold = minimum_jump_m
    events = []
    for index, magnitude in enumerate(steps, start=1):
        if np.isfinite(magnitude) and magnitude > threshold:
            events.append({
                "epoch_ns": rows[index]["epoch_ns"],
                "epoch_utc": rows[index]["epoch_utc"],
                "previous_epoch_ns": rows[index - 1]["epoch_ns"],
                "jump_3d_m": float(magnitude),
                "north_jump_m": rows[index]["north_m"] - rows[index - 1]["north_m"],
                "east_jump_m": rows[index]["east_m"] - rows[index - 1]["east_m"],
                "up_jump_m": rows[index]["up_m"] - rows[index - 1]["up_m"],
                "threshold_m": threshold,
            })
    return events, threshold


def fixed_segments(
    rows: list[dict[str, Any]], field: str, max_gap_seconds: float
) -> list[dict[str, Any]]:
    segments: list[dict[str, Any]] = []
    start: int | None = None
    for index, row in enumerate(rows):
        active = bool(row[field])
        gap = (
            (row["epoch_ns"] - rows[index - 1]["epoch_ns"]) / 1e9
            if index > 0 else 0.0
        )
        if active and (start is None or (index > 0 and gap > max_gap_seconds)):
            if start is not None:
                segments.append(_segment(rows, start, index - 1, field))
            start = index
        elif not active and start is not None:
            segments.append(_segment(rows, start, index - 1, field))
            start = None
    if start is not None:
        segments.append(_segment(rows, start, len(rows) - 1, field))
    return segments


def _segment(rows: list[dict[str, Any]], start: int, end: int, field: str) -> dict[str, Any]:
    return {
        "segment_type": field,
        "start_epoch_ns": rows[start]["epoch_ns"],
        "start_epoch_utc": rows[start]["epoch_utc"],
        "end_epoch_ns": rows[end]["epoch_ns"],
        "end_epoch_utc": rows[end]["epoch_utc"],
        "duration_seconds": (rows[end]["epoch_ns"] - rows[start]["epoch_ns"]) / 1e9,
        "epoch_count": end - start + 1,
    }


def loss_reconvergence_events(
    rows: list[dict[str, Any]], horizontal_threshold_m: float, vertical_threshold_m: float,
    sustain_seconds: float, max_gap_seconds: float,
) -> list[dict[str, Any]]:
    events: list[dict[str, Any]] = []
    for index in range(1, len(rows)):
        if not rows[index - 1]["any_fix"] or rows[index]["any_fix"]:
            continue
        tail = rows[index:]
        epochs = np.asarray([row["epoch_ns"] for row in tail], dtype=np.int64)
        horizontal = np.asarray([row["horizontal_m"] for row in tail], dtype=float)
        vertical = np.abs(np.asarray([row["up_m"] for row in tail], dtype=float))
        seconds = sustained_convergence_seconds(
            epochs, horizontal, vertical, horizontal_threshold_m, vertical_threshold_m,
            sustain_seconds, max_gap_seconds,
        )
        fixed_epoch = next((row["epoch_ns"] for row in tail if row["any_fix"]), None)
        events.append({
            "loss_epoch_ns": rows[index]["epoch_ns"],
            "loss_epoch_utc": rows[index]["epoch_utc"],
            "refix_seconds": (
                (fixed_epoch - rows[index]["epoch_ns"]) / 1e9 if fixed_epoch is not None else float("nan")
            ),
            "position_reconvergence_seconds": seconds,
        })
    return events
