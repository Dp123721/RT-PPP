"""Compares SP3 orbit and clock products on common epochs."""

from __future__ import annotations

from bisect import bisect_left
from itertools import combinations
from pathlib import Path
from typing import Any

import numpy as np

from ..core.coordinates import ecef_delta_to_rac
from ..core.statistics import describe
from ..core.time import unix_ns_to_datetime
from ..contracts import validate_metrics
from ..io.sp3_clk import load_product
from ..io.tables import json_value, write_csv, write_json
from ..models import ClockPoint, OrbitPoint, Product
from ..plotting.figures import (
    orbit_figure,
    orbit_histogram_figure,
    orbit_satellite_rms_figure,
)
from ..provenance import write_manifest
from ..reports.html import write_html_report


SPEED_OF_LIGHT_MPS = 299792458.0
SISRE_TRANSVERSE_WEIGHT = 1.0 / 49.0


def _coverage_rate(count: int, total: int) -> float:
    return count / total if total else float("nan")


def _barycentric_interpolate(times_s: np.ndarray, values: np.ndarray) -> np.ndarray:
    if np.any(times_s == 0.0):
        return values[int(np.flatnonzero(times_s == 0.0)[0])]
    weights = np.ones(times_s.size)
    for index in range(times_s.size):
        weights[index] = 1.0 / np.prod(times_s[index] - np.delete(times_s, index))
    scaled = weights / (-times_s)
    return np.sum(scaled[:, None] * values, axis=0) / np.sum(scaled)


class OrbitInterpolator:
    def __init__(self, points: list[OrbitPoint]):
        self.epoch_ns = np.asarray([point.epoch_ns for point in points], dtype=np.int64)
        self.positions = np.asarray([point.position_m for point in points], dtype=float)
        supplied = [point.velocity_mps for point in points]
        if len(points) >= 2:
            seconds = (self.epoch_ns - self.epoch_ns[0]) / 1e9
            edge_order = 2 if len(points) >= 3 else 1
            estimated = np.gradient(self.positions, seconds, axis=0, edge_order=edge_order)
        else:
            estimated = np.full_like(self.positions, np.nan)
        self.velocities = np.asarray(
            [velocity if velocity is not None else estimated[index] for index, velocity in enumerate(supplied)]
        )

    def at(self, epoch_ns: int, point_count: int, max_gap_s: float) -> tuple[np.ndarray, np.ndarray] | None:
        if self.epoch_ns.size < 2 or epoch_ns < self.epoch_ns[0] or epoch_ns > self.epoch_ns[-1]:
            return None
        insertion = int(np.searchsorted(self.epoch_ns, epoch_ns))
        if insertion < self.epoch_ns.size and self.epoch_ns[insertion] == epoch_ns:
            return self.positions[insertion], self.velocities[insertion]
        count = min(max(2, point_count), self.epoch_ns.size)
        start = max(0, insertion - count // 2)
        start = min(start, self.epoch_ns.size - count)
        selected_epochs = self.epoch_ns[start : start + count]
        if np.max(np.diff(selected_epochs)) / 1e9 > max_gap_s:
            return None
        relative_s = (selected_epochs - epoch_ns) / 1e9
        position = _barycentric_interpolate(relative_s, self.positions[start : start + count])
        velocity = _barycentric_interpolate(relative_s, self.velocities[start : start + count])
        return position, velocity


def _clock_at(points: list[ClockPoint], epoch_ns: int, max_gap_s: float) -> float:
    if not points:
        return float("nan")
    epochs = [point.epoch_ns for point in points]
    index = bisect_left(epochs, epoch_ns)
    if index < len(points) and points[index].epoch_ns == epoch_ns:
        return points[index].clock_s
    if index == 0 or index >= len(points):
        return float("nan")
    before, after = points[index - 1], points[index]
    interval_s = (after.epoch_ns - before.epoch_ns) / 1e9
    if interval_s <= 0.0 or interval_s > max_gap_s:
        return float("nan")
    fraction = (epoch_ns - before.epoch_ns) / (after.epoch_ns - before.epoch_ns)
    return before.clock_s + fraction * (after.clock_s - before.clock_s)


def sisre_m(radial_m: float, along_m: float, cross_m: float, clock_m: float) -> float:
    """Return the standard orbit-plus-clock SISRE in metres.

    ``clock_m`` is the candidate-minus-reference satellite-clock residual after
    removal of the per-constellation, per-epoch datum.  The sign convention is
    therefore consistent with the candidate-minus-reference RAC residuals.
    """
    values = np.asarray([radial_m, along_m, cross_m, clock_m], dtype=float)
    if not np.all(np.isfinite(values)):
        return float("nan")
    radial, along, cross, clock = values
    return float(np.sqrt((radial - clock) ** 2 + SISRE_TRANSVERSE_WEIGHT * (along**2 + cross**2)))


def compare_product(
    candidate: Product,
    reference: Product,
    options: dict[str, Any],
    comparison_scope: str,
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    interpolation_points = int(options.get("orbit_interpolation_points", 9))
    max_orbit_gap_s = float(options.get("max_orbit_gap_seconds", 900.0))
    max_clock_gap_s = float(options.get("max_clock_gap_seconds", 120.0))
    interpolators = {
        satellite: OrbitInterpolator(points) for satellite, points in reference.orbits.items()
    }
    rows: list[dict[str, Any]] = []
    candidate_sample_count = sum(len(points) for points in candidate.orbits.values())
    rejection_counts: dict[str, int] = {}
    satellite_rejection_counts: dict[str, dict[str, int]] = {}

    def reject(reason: str, satellite: str, count: int = 1) -> None:
        rejection_counts[reason] = rejection_counts.get(reason, 0) + count
        satellite_counts = satellite_rejection_counts.setdefault(satellite, {})
        satellite_counts[reason] = satellite_counts.get(reason, 0) + count

    for satellite, candidate_points in candidate.orbits.items():
        interpolator = interpolators.get(satellite)
        if interpolator is None:
            reject("missing_reference_satellite", satellite, len(candidate_points))
            continue
        for point in candidate_points:
            if interpolator.epoch_ns.size < 2:
                reject("insufficient_reference_points", satellite)
                continue
            if point.epoch_ns < interpolator.epoch_ns[0] or point.epoch_ns > interpolator.epoch_ns[-1]:
                reject("outside_reference_span", satellite)
                continue
            state = interpolator.at(point.epoch_ns, interpolation_points, max_orbit_gap_s)
            if state is None:
                reject("orbit_gap_exceeded", satellite)
                continue
            reference_position, reference_velocity = state
            delta = point.position_m - reference_position
            try:
                rac = ecef_delta_to_rac(delta, reference_position, reference_velocity)
            except ValueError:
                reject("invalid_rac_basis", satellite)
                continue
            candidate_clock = _clock_at(candidate.clocks.get(satellite, []), point.epoch_ns, max_clock_gap_s)
            reference_clock = _clock_at(reference.clocks.get(satellite, []), point.epoch_ns, max_clock_gap_s)
            clock_m = (
                (candidate_clock - reference_clock) * SPEED_OF_LIGHT_MPS
                if np.isfinite(candidate_clock) and np.isfinite(reference_clock)
                else float("nan")
            )
            if not np.isfinite(candidate_clock) and not np.isfinite(reference_clock):
                reject("clock_missing_both", satellite)
            elif not np.isfinite(candidate_clock):
                reject("clock_missing_candidate", satellite)
            elif not np.isfinite(reference_clock):
                reject("clock_missing_reference", satellite)
            rows.append(
                {
                    "comparison_scope": comparison_scope,
                    "candidate_id": candidate.product_id,
                    "reference_id": reference.product_id,
                    "epoch_ns": point.epoch_ns,
                    "epoch_utc": unix_ns_to_datetime(point.epoch_ns).isoformat(),
                    "satellite": satellite,
                    "system": satellite[:1],
                    "dx_m": float(delta[0]),
                    "dy_m": float(delta[1]),
                    "dz_m": float(delta[2]),
                    "radial_m": float(rac[0]),
                    "along_m": float(rac[1]),
                    "cross_m": float(rac[2]),
                    "orbit_3d_m": float(np.linalg.norm(delta)),
                    "clock_raw_m": float(clock_m),
                    "clock_datum_removed_m": float("nan"),
                }
            )
    datum_groups: dict[tuple[int, str], list[int]] = {}
    for index, row in enumerate(rows):
        datum_groups.setdefault((row["epoch_ns"], row["system"]), []).append(index)
    for indices in datum_groups.values():
        values = np.asarray([rows[index]["clock_raw_m"] for index in indices], dtype=float)
        finite = values[np.isfinite(values)]
        if finite.size == 0:
            continue
        datum = float(np.median(finite))
        for index in indices:
            value = rows[index]["clock_raw_m"]
            if np.isfinite(value):
                rows[index]["clock_datum_removed_m"] = value - datum
    for row in rows:
        row["sisre_m"] = sisre_m(
            row["radial_m"],
            row["along_m"],
            row["cross_m"],
            row["clock_datum_removed_m"],
        )
    clock_compared_count = sum(np.isfinite(row["clock_raw_m"]) for row in rows)
    rows_by_satellite: dict[str, list[dict[str, Any]]] = {}
    for row in rows:
        rows_by_satellite.setdefault(str(row["satellite"]), []).append(row)
    satellite_metrics: list[dict[str, Any]] = []
    metric_names = (
        "radial_m",
        "along_m",
        "cross_m",
        "orbit_3d_m",
        "clock_raw_m",
        "clock_datum_removed_m",
        "sisre_m",
    )
    for satellite, candidate_points in sorted(candidate.orbits.items()):
        selected = rows_by_satellite.get(satellite, [])
        satellite_clock_count = sum(np.isfinite(row["clock_raw_m"]) for row in selected)
        satellite_metrics.append(
            {
                "comparison_scope": comparison_scope,
                "candidate_id": candidate.product_id,
                "reference_id": reference.product_id,
                "system": satellite[:1],
                "satellite": satellite,
                "candidate_sample_count": len(candidate_points),
                "compared_orbit_count": len(selected),
                "clock_compared_count": int(satellite_clock_count),
                "orbit_coverage_rate": _coverage_rate(len(selected), len(candidate_points)),
                "clock_coverage_rate": _coverage_rate(
                    int(satellite_clock_count), len(candidate_points)
                ),
                "rejection_counts": satellite_rejection_counts.get(satellite, {}),
                "statistics": {
                    metric: describe(row[metric] for row in selected)
                    for metric in metric_names
                },
            }
        )
    coverage = {
        "comparison_scope": comparison_scope,
        "candidate_id": candidate.product_id,
        "reference_id": reference.product_id,
        "candidate_sample_count": candidate_sample_count,
        "compared_orbit_count": len(rows),
        "clock_compared_count": int(clock_compared_count),
        "orbit_coverage_rate": _coverage_rate(len(rows), candidate_sample_count),
        "clock_coverage_rate": _coverage_rate(int(clock_compared_count), candidate_sample_count),
        "rejection_counts": rejection_counts,
        "_satellite_metrics": satellite_metrics,
    }
    return rows, coverage


def _flatten_satellite_metrics(metrics: list[dict[str, Any]]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for metric in metrics:
        row = {key: value for key, value in metric.items() if key != "statistics"}
        for field, statistics in metric["statistics"].items():
            for name, value in statistics.items():
                row[f"{field}_{name}"] = value
        rows.append(row)
    return rows


def _metadata_warnings(candidate: Product, reference: Product) -> list[str]:
    prefix = f"{candidate.product_id} vs {reference.product_id}"
    warnings: list[str] = []
    if candidate.time_scale != reference.time_scale:
        warnings.append(
            f"{prefix}: incompatible time scales "
            f"({candidate.time_scale}/{reference.time_scale})"
        )
    if candidate.orbit_reference == "UNKNOWN" or reference.orbit_reference == "UNKNOWN":
        warnings.append(f"{prefix}: APC/CoM semantics are unknown")
    elif candidate.orbit_reference != reference.orbit_reference:
        warnings.append(
            f"{prefix}: incompatible orbit references "
            f"({candidate.orbit_reference}/{reference.orbit_reference})"
        )
    if candidate.reference_frame == "UNKNOWN" or reference.reference_frame == "UNKNOWN":
        warnings.append(f"{prefix}: reference frame is unknown")
    elif candidate.reference_frame != reference.reference_frame:
        warnings.append(
            f"{prefix}: incompatible reference frames "
            f"({candidate.reference_frame}/{reference.reference_frame})"
        )
    return warnings


def _summaries(rows: list[dict[str, Any]], scope: str) -> list[dict[str, Any]]:
    metrics = (
        "radial_m", "along_m", "cross_m", "orbit_3d_m", "clock_raw_m",
        "clock_datum_removed_m", "sisre_m",
    )
    grouped: dict[tuple[str, str, str], list[dict[str, Any]]] = {}
    for row in rows:
        grouped.setdefault((row["candidate_id"], row["reference_id"], row["system"]), []).append(row)
    output = []
    for (candidate, reference, system), selected in sorted(grouped.items()):
        result: dict[str, Any] = {
            "scope": scope,
            "candidate_id": candidate,
            "reference_id": reference,
            "system": system,
            "sample_count": len(selected),
            "satellite_count": len({row["satellite"] for row in selected}),
        }
        for metric in metrics:
            for name, value in describe(row[metric] for row in selected).items():
                result[f"{metric}_{name}"] = value
        output.append(result)
    return output


def run_orbit_analysis(config: dict[str, Any], config_path: Path, output_directory: Path) -> dict[str, Any]:
    reference = load_product(config["reference"], config_path.parent)
    candidates = [load_product(item, config_path.parent) for item in config.get("candidates", [])]
    if not candidates:
        raise ValueError("Orbit analysis requires at least one candidate product")
    options = config.get("options", {})
    warnings: list[str] = []
    for candidate in candidates:
        warnings.extend(_metadata_warnings(candidate, reference))

    rows: list[dict[str, Any]] = []
    coverage: list[dict[str, Any]] = []
    satellite_metrics: list[dict[str, Any]] = []
    by_candidate: dict[str, list[dict[str, Any]]] = {}
    for candidate in candidates:
        compared, comparison_coverage = compare_product(candidate, reference, options, "reference")
        satellite_metrics.extend(comparison_coverage.pop("_satellite_metrics"))
        by_candidate[candidate.product_id] = compared
        rows.extend(compared)
        coverage.append(comparison_coverage)

    common_keys: set[tuple[int, str]] = set()
    for index, candidate in enumerate(candidates):
        keys = {(row["epoch_ns"], row["satellite"]) for row in by_candidate[candidate.product_id]}
        common_keys = keys if index == 0 else common_keys & keys
    common_rows = [row for row in rows if (row["epoch_ns"], row["satellite"]) in common_keys]

    pairwise_rows: list[dict[str, Any]] = []
    if bool(options.get("pairwise_centers", True)):
        for first, second in combinations(candidates, 2):
            compared, comparison_coverage = compare_product(
                first, second, options, "pairwise_center"
            )
            satellite_metrics.extend(comparison_coverage.pop("_satellite_metrics"))
            pairwise_rows.extend(compared)
            coverage.append(comparison_coverage)
            warnings.extend(_metadata_warnings(first, second))
    rows.extend(pairwise_rows)

    summaries = _summaries([row for row in rows if row["comparison_scope"] == "reference"], "all")
    summaries.extend(_summaries(common_rows, "common"))
    summaries.extend(_summaries(pairwise_rows, "pairwise"))
    output_directory.mkdir(parents=True, exist_ok=True)
    write_csv(output_directory / "orbit_clock_residuals.csv", rows)
    write_csv(output_directory / "orbit_clock_summary.csv", summaries)
    write_csv(
        output_directory / "orbit_clock_satellite_summary.csv",
        _flatten_satellite_metrics(satellite_metrics),
    )
    summary = {
        "reference": reference.product_id,
        "candidates": [candidate.product_id for candidate in candidates],
        "products": [
            {
                "id": product.product_id,
                "role": "reference" if product is reference else "candidate",
                "time_scale": product.time_scale,
                "orbit_reference": product.orbit_reference,
                "reference_frame": product.reference_frame,
            }
            for product in [reference, *candidates]
        ],
        "row_count": len(rows),
        "common_sample_count": len(common_keys),
        "metrics": summaries,
        "satellite_metrics": satellite_metrics,
        "coverage": coverage,
        "warnings": warnings,
    }
    validate_metrics("orbit_clock", json_value(summary))
    write_json(output_directory / "metrics.json", summary)
    figures = [
        orbit_histogram_figure(rows, output_directory / "orbit_clock_histograms.png"),
        orbit_satellite_rms_figure(rows, output_directory / "orbit_clock_satellite_rms.png"),
        orbit_figure(rows, output_directory / "orbit_clock_timeseries.png"),
    ]
    figures = [figure for figure in figures if figure is not None]
    write_html_report(output_directory / "report.html", "Orbit and Clock Analysis", summary, figures, warnings)
    inputs = [Path(path) for product in [reference, *candidates] for path in product.input_paths]
    write_manifest(output_directory, "orbit_clock", config_path, config, inputs, warnings)
    return summary
