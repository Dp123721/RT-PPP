"""Runs a configured PPP analysis and assembles its result document."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np

from ..core.coordinates import covariance_ecef_to_enu, ecef_delta_to_enu
from ..core.statistics import describe, sustained_convergence_seconds
from ..core.time import unix_ns_to_datetime
from ..contracts import validate_metrics
from ..io.ppp_json import read_ppp_json
from ..io.tables import json_value, write_csv, write_json
from ..models import MissingReason
from ..plotting.figures import (
    ppp_error_histogram_figure,
    ppp_figure,
    ppp_residual_figure,
    ppp_ztd_figure,
)
from ..provenance import write_manifest
from ..reports.html import write_html_report
from .reference import load_reference
from .residuals import read_residuals
from .statistics import (
    bootstrap_mean_ci,
    detect_robust_jumps,
    fixed_segments,
    loss_reconvergence_events,
    paired_sign_test_pvalue,
)


def _rate(count: int, total: int) -> float:
    return count / total if total else float("nan")


def _finite_inverse_nees(error_enu: np.ndarray, covariance_enu: np.ndarray) -> float:
    if not np.all(np.isfinite(error_enu)) or not np.all(np.isfinite(covariance_enu)):
        return float("nan")
    try:
        if np.min(np.linalg.eigvalsh(covariance_enu)) <= 0.0:
            return float("nan")
        return float(error_enu @ np.linalg.solve(covariance_enu, error_enu))
    except np.linalg.LinAlgError:
        return float("nan")


def _validate_ar_epoch(epoch: Any, row_number: int) -> None:
    if not 0 <= epoch.fixed_count <= epoch.eligible_count <= epoch.used_count:
        raise ValueError(
            f"PPP epoch {row_number}: fixed <= eligible <= used is violated "
            f"({epoch.fixed_count}, {epoch.eligible_count}, {epoch.used_count})"
        )
    if len(epoch.satellite_sets) == 3:
        lengths = [len(value) if isinstance(value, list) else -1 for value in epoch.satellite_sets]
        expected = [epoch.used_count, epoch.eligible_count, epoch.fixed_count]
        if lengths != expected:
            raise ValueError(
                f"PPP epoch {row_number}: satellite set lengths {lengths} do not match {expected}"
            )
    inconsistent = (
        (epoch.ar_status in {"disabled", "float"} and epoch.fixed_count != 0)
        or (
            epoch.ar_status == "partial_fix"
            and not 0 < epoch.fixed_count < epoch.eligible_count
        )
        or (
            epoch.ar_status == "fixed"
            and not (epoch.eligible_count > 0 and epoch.fixed_count == epoch.eligible_count)
        )
    )
    if inconsistent:
        raise ValueError(
            f"PPP epoch {row_number}: AR status {epoch.ar_status!r} is inconsistent with "
            f"fixed={epoch.fixed_count}, eligible={epoch.eligible_count}"
        )


def _resolve_path(value: str, config_path: Path) -> Path:
    path = Path(value).expanduser()
    if not path.is_absolute():
        path = config_path.parent / path
    return path.resolve()


def _residual_groups(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    groups: dict[tuple[str, str, str], list[dict[str, Any]]] = {}
    for row in rows:
        key = (str(row["dataset_id"]), str(row["observation_type"]), str(row["signal"]))
        groups.setdefault(key, []).append(row)
    result = []
    for (dataset_id, observation_type, signal), values in sorted(groups.items()):
        adopted = [row for row in values if row["adopted"]]
        elevation = np.asarray([row["elevation_deg"] for row in adopted], dtype=float)
        absolute = np.abs(np.asarray([row["postfit_m"] for row in adopted], dtype=float))
        correlation = (
            float(np.corrcoef(elevation, absolute)[0, 1])
            if elevation.size >= 2 and np.std(elevation) > 0.0 and np.std(absolute) > 0.0
            else float("nan")
        )
        result.append({
            "dataset_id": dataset_id,
            "observation_type": observation_type,
            "signal": signal,
            "total_count": len(values),
            "adopted_count": len(adopted),
            "rejected_count": len(values) - len(adopted),
            "adoption_rate": _rate(len(adopted), len(values)),
            "prefit_m": describe(row["prefit_m"] for row in adopted),
            "postfit_m": describe(row["postfit_m"] for row in adopted),
            "normalized_prefit": describe(row["normalized_prefit"] for row in adopted),
            "normalized_postfit": describe(row["normalized_postfit"] for row in adopted),
            "elevation_abs_postfit_correlation": correlation,
            "outlier_count": sum(bool(row["outlier"]) for row in adopted),
        })
    return result


def run_ppp_analysis(
    config: dict[str, Any], config_path: Path, output_directory: Path
) -> dict[str, Any]:
    datasets = config.get("datasets", [])
    if not isinstance(datasets, list) or not datasets:
        raise ValueError("PPP analysis requires at least one dataset")
    options = config.get("options", {})
    horizontal_threshold_m = float(options.get("horizontal_threshold_m", 0.1))
    vertical_threshold_m = float(options.get("vertical_threshold_m", 0.2))
    sustain_seconds = float(options.get("sustain_seconds", 600.0))
    convergence_gap_factor = float(options.get("convergence_gap_factor", 1.5))
    configured_max_gap_s = float(options.get("convergence_max_gap_seconds", 0.0))
    jump_mad_factor = float(options.get("jump_mad_factor", 8.0))
    jump_minimum_m = float(options.get("jump_minimum_m", 0.05))
    bootstrap_samples = int(options.get("bootstrap_samples", 2000))
    bootstrap_confidence = float(options.get("bootstrap_confidence", 0.95))
    residual_outlier_sigma = float(options.get("residual_outlier_sigma", 4.0))
    if convergence_gap_factor <= 1.0:
        raise ValueError("convergence_gap_factor must be greater than 1")

    rows: list[dict[str, Any]] = []
    residual_rows: list[dict[str, Any]] = []
    jump_rows: list[dict[str, Any]] = []
    segment_rows: list[dict[str, Any]] = []
    reconvergence_rows: list[dict[str, Any]] = []
    summaries: list[dict[str, Any]] = []
    input_paths: list[Path] = []
    warnings: list[str] = []

    for dataset in datasets:
        dataset_id = str(dataset.get("id", "")).strip()
        if not dataset_id:
            raise ValueError("Every PPP dataset requires an id")
        path = _resolve_path(str(dataset.get("path", "")), config_path)
        input_paths.append(path)
        metadata, epochs = read_ppp_json(path)
        reference = load_reference(dataset, config_path)
        if reference.source_path:
            input_paths.append(reference.source_path)
        configured_label = str(dataset.get("algorithm_label", "")).strip()
        dataset_rows: list[dict[str, Any]] = []
        for row_number, epoch in enumerate(epochs):
            _validate_ar_epoch(epoch, row_number)
            truth = reference.at(epoch.epoch_ns)
            truth_available = bool(np.all(np.isfinite(truth)))
            if truth_available:
                error_enu = ecef_delta_to_enu(epoch.ecef_m - truth, truth)
                covariance_enu = covariance_ecef_to_enu(epoch.covariance_ecef_m2, truth)
                sigma_enu = np.sqrt(np.maximum(0.0, np.diag(covariance_enu)))
                normalized = np.divide(
                    error_enu, sigma_enu, out=np.full(3, np.nan), where=sigma_enu > 0.0
                )
                nees = _finite_inverse_nees(error_enu, covariance_enu)
            else:
                error_enu = np.full(3, np.nan)
                sigma_enu = np.full(3, np.nan)
                normalized = np.full(3, np.nan)
                nees = float("nan")
            computed_neu = np.asarray([error_enu[1], error_enu[0], error_enu[2]])
            sigma_neu = np.asarray([sigma_enu[1], sigma_enu[0], sigma_enu[2]])
            stored_delta_neu = epoch.stored_enu_m - computed_neu
            horizontal = float(np.hypot(computed_neu[0], computed_neu[1]))
            row = {
                "dataset_id": dataset_id,
                "algorithm_label": configured_label or epoch.algorithm,
                "epoch_ns": epoch.epoch_ns,
                "epoch_utc": unix_ns_to_datetime(epoch.epoch_ns).isoformat(),
                "algorithm": epoch.algorithm,
                "ar_status": epoch.ar_status,
                "reference_x_m": truth[0],
                "reference_y_m": truth[1],
                "reference_z_m": truth[2],
                "north_m": computed_neu[0],
                "east_m": computed_neu[1],
                "up_m": computed_neu[2],
                "horizontal_m": horizontal,
                "error_3d_m": float(np.linalg.norm(computed_neu)),
                "north_sigma_m": sigma_neu[0],
                "east_sigma_m": sigma_neu[1],
                "up_sigma_m": sigma_neu[2],
                "north_normalized": normalized[1],
                "east_normalized": normalized[0],
                "up_normalized": normalized[2],
                "nees": nees,
                "stored_north_m": epoch.stored_enu_m[0],
                "stored_east_m": epoch.stored_enu_m[1],
                "stored_up_m": epoch.stored_enu_m[2],
                "stored_minus_computed_north_m": stored_delta_neu[0],
                "stored_minus_computed_east_m": stored_delta_neu[1],
                "stored_minus_computed_up_m": stored_delta_neu[2],
                "ztd_m": epoch.ztd_m,
                "ztd_std_m": epoch.ztd_std_m,
                "wet_residual_m": epoch.wet_residual_m,
                "used_count": epoch.used_count,
                "eligible_count": epoch.eligible_count,
                "fixed_count": epoch.fixed_count,
                "hdop": epoch.hdop,
                "any_fix": int(epoch.fixed_count > 0),
                "full_fix": int(
                    epoch.eligible_count > 0 and epoch.fixed_count == epoch.eligible_count
                ),
                "ar_data": epoch.ar_data,
            }
            dataset_rows.append(row)
            rows.append(row)

        epoch_ns = np.asarray([row["epoch_ns"] for row in dataset_rows], dtype=np.int64)
        horizontal = np.asarray([row["horizontal_m"] for row in dataset_rows], dtype=float)
        vertical = np.abs(np.asarray([row["up_m"] for row in dataset_rows], dtype=float))
        gaps = np.diff(epoch_ns) / 1e9
        inferred_interval_s = (
            float(np.median(gaps[gaps > 0.0])) if np.any(gaps > 0.0) else float("nan")
        )
        convergence_max_gap_s = configured_max_gap_s
        if convergence_max_gap_s <= 0.0 and np.isfinite(inferred_interval_s):
            convergence_max_gap_s = inferred_interval_s * convergence_gap_factor
        if convergence_max_gap_s <= 0.0:
            convergence_max_gap_s = float("nan")
        segment_gap_s = convergence_max_gap_s if np.isfinite(convergence_max_gap_s) else 1.0

        dataset_jumps, jump_threshold = detect_robust_jumps(
            dataset_rows, jump_mad_factor, jump_minimum_m
        )
        dataset_jumps = [{"dataset_id": dataset_id, **row} for row in dataset_jumps]
        dataset_segments = [
            {"dataset_id": dataset_id, **row}
            for field in ("any_fix", "full_fix")
            for row in fixed_segments(dataset_rows, field, segment_gap_s)
        ]
        dataset_reconvergence = [
            {"dataset_id": dataset_id, **row}
            for row in loss_reconvergence_events(
                dataset_rows,
                horizontal_threshold_m,
                vertical_threshold_m,
                sustain_seconds,
                segment_gap_s,
            )
        ]
        jump_rows.extend(dataset_jumps)
        segment_rows.extend(dataset_segments)
        reconvergence_rows.extend(dataset_reconvergence)

        dataset_residuals: list[dict[str, Any]] = []
        residual_value = str(dataset.get("residual_directory", "")).strip()
        if residual_value:
            residual_path = _resolve_path(residual_value, config_path)
            source_id = str(dataset.get("residual_source_id", "")).strip() or None
            raw_residuals = read_residuals(residual_path, source_id)
            input_paths.extend(
                sorted(residual_path.glob("ppp_residual_*.csv"))
                if residual_path.is_dir()
                else [residual_path]
            )
            for residual in raw_residuals:
                normalized_prefit = residual["prefit_m"] / residual["sigma_m"]
                normalized_postfit = residual["postfit_m"] / residual["sigma_m"]
                dataset_residuals.append({
                    "dataset_id": dataset_id,
                    **residual,
                    "normalized_prefit": normalized_prefit,
                    "normalized_postfit": normalized_postfit,
                    "nis": normalized_prefit * normalized_prefit,
                    "outlier": bool(
                        residual["adopted"]
                        and abs(normalized_postfit) > residual_outlier_sigma
                    ),
                })
            residual_rows.extend(dataset_residuals)
        else:
            warnings.append(
                f"{dataset_id}: final-iteration PPP residual telemetry is unavailable; "
                "residual diagnostics remain unknown."
            )

        fixed_total = sum(row["fixed_count"] for row in dataset_rows)
        eligible_total = sum(row["eligible_count"] for row in dataset_rows)
        used_total = sum(row["used_count"] for row in dataset_rows)
        finite_solution_count = sum(
            np.isfinite(row["north_m"])
            and np.isfinite(row["east_m"])
            and np.isfinite(row["up_m"])
            for row in dataset_rows
        )
        any_segments = [row for row in dataset_segments if row["segment_type"] == "any_fix"]
        full_segments = [row for row in dataset_segments if row["segment_type"] == "full_fix"]
        adopted_residuals = [row for row in dataset_residuals if row["adopted"]]
        nees_values = [row["nees"] for row in dataset_rows]
        finite_nees = [value for value in nees_values if np.isfinite(value)]
        summaries.append({
            "dataset_id": dataset_id,
            "algorithm_label": configured_label
            or (dataset_rows[0]["algorithm"] if dataset_rows else ""),
            "station": metadata.get("station", {}).get("id"),
            "compact_version": int(metadata.get("version", 0)),
            "reference_mode": reference.mode,
            "epoch_count": len(dataset_rows),
            "finite_solution_count": int(finite_solution_count),
            "solution_coverage_rate": _rate(finite_solution_count, len(dataset_rows)),
            "reference_ecef_m": reference.base_ecef_m,
            "north_m": describe(row["north_m"] for row in dataset_rows),
            "east_m": describe(row["east_m"] for row in dataset_rows),
            "up_m": describe(row["up_m"] for row in dataset_rows),
            "horizontal_m": describe(row["horizontal_m"] for row in dataset_rows),
            "error_3d_m": describe(row["error_3d_m"] for row in dataset_rows),
            "ztd_m": describe(row["ztd_m"] for row in dataset_rows),
            "wet_residual_m": describe(row["wet_residual_m"] for row in dataset_rows),
            "ztd_std_m": describe(row["ztd_std_m"] for row in dataset_rows),
            "hdop": describe(row["hdop"] for row in dataset_rows),
            "north_normalized": describe(row["north_normalized"] for row in dataset_rows),
            "east_normalized": describe(row["east_normalized"] for row in dataset_rows),
            "up_normalized": describe(row["up_normalized"] for row in dataset_rows),
            "nees": describe(nees_values),
            "nees_95_coverage_rate": _rate(
                sum(value <= 7.8147279 for value in finite_nees), len(finite_nees)
            ),
            "stored_minus_computed_neu_m": {
                "north": describe(
                    row["stored_minus_computed_north_m"] for row in dataset_rows
                ),
                "east": describe(
                    row["stored_minus_computed_east_m"] for row in dataset_rows
                ),
                "up": describe(row["stored_minus_computed_up_m"] for row in dataset_rows),
            },
            "inferred_interval_seconds": inferred_interval_s,
            "convergence_max_gap_seconds": convergence_max_gap_s,
            "convergence_seconds": sustained_convergence_seconds(
                epoch_ns,
                horizontal,
                vertical,
                horizontal_threshold_m,
                vertical_threshold_m,
                sustain_seconds,
                convergence_max_gap_s,
            ),
            "any_fix_epoch_rate": _rate(
                sum(row["any_fix"] for row in dataset_rows), len(dataset_rows)
            ),
            "full_fix_epoch_rate": _rate(
                sum(row["full_fix"] for row in dataset_rows), len(dataset_rows)
            ),
            "fixed_over_eligible_rate": _rate(fixed_total, eligible_total),
            "fixed_over_used_rate": _rate(fixed_total, used_total),
            "ar_consistency_violation_count": 0,
            "jump_count": len(dataset_jumps),
            "jump_threshold_m": jump_threshold,
            "any_fix_segment_count": len(any_segments),
            "full_fix_segment_count": len(full_segments),
            "longest_any_fix_seconds": max(
                (row["duration_seconds"] for row in any_segments), default=float("nan")
            ),
            "longest_full_fix_seconds": max(
                (row["duration_seconds"] for row in full_segments), default=float("nan")
            ),
            "fix_loss_count": len(dataset_reconvergence),
            "refix_seconds": describe(row["refix_seconds"] for row in dataset_reconvergence),
            "position_reconvergence_seconds": describe(
                row["position_reconvergence_seconds"] for row in dataset_reconvergence
            ),
            "residual_analysis_available": bool(dataset_residuals),
            "residual_unavailable_reason": (
                None if dataset_residuals else MissingReason.UNKNOWN.value
            ),
            "residual_count": len(dataset_residuals),
            "residual_nis": describe(row["nis"] for row in adopted_residuals),
        })

    by_dataset = {
        str(dataset["id"]): {
            row["epoch_ns"]: row
            for row in rows
            if row["dataset_id"] == dataset["id"]
        }
        for dataset in datasets
    }
    common_epochs = (
        set.intersection(*(set(values) for values in by_dataset.values()))
        if len(datasets) >= 2
        else set()
    )
    common_epoch_count = len(common_epochs)
    common_epoch_coverage = []
    if len(datasets) >= 2:
        for dataset in datasets:
            dataset_id = str(dataset["id"])
            common_epoch_coverage.append({
                "dataset_id": dataset_id,
                "epoch_count": len(by_dataset[dataset_id]),
                "common_epoch_count": common_epoch_count,
                "coverage_rate": _rate(common_epoch_count, len(by_dataset[dataset_id])),
            })

    paired_rows: list[dict[str, Any]] = []
    paired_comparisons: list[dict[str, Any]] = []
    if len(datasets) >= 2:
        first_id = str(datasets[0]["id"])
        for pair_index, other in enumerate(datasets[1:], start=1):
            other_id = str(other["id"])
            pair_values: dict[str, list[float]] = {"horizontal_m": [], "error_3d_m": []}
            for epoch_ns in sorted(common_epochs):
                first = by_dataset[first_id][epoch_ns]
                second = by_dataset[other_id][epoch_ns]
                record = {
                    "epoch_ns": epoch_ns,
                    "epoch_utc": first["epoch_utc"],
                    "first_id": first_id,
                    "second_id": other_id,
                    "north_difference_m": first["north_m"] - second["north_m"],
                    "east_difference_m": first["east_m"] - second["east_m"],
                    "up_difference_m": first["up_m"] - second["up_m"],
                    "horizontal_difference_m": (
                        first["horizontal_m"] - second["horizontal_m"]
                    ),
                    "error_3d_difference_m": (
                        first["error_3d_m"] - second["error_3d_m"]
                    ),
                    "fixed_count_difference": (
                        first["fixed_count"] - second["fixed_count"]
                    ),
                }
                paired_rows.append(record)
                pair_values["horizontal_m"].append(record["horizontal_difference_m"])
                pair_values["error_3d_m"].append(record["error_3d_difference_m"])
            for metric, values in pair_values.items():
                low, high = bootstrap_mean_ci(
                    values,
                    bootstrap_samples,
                    bootstrap_confidence,
                    20260817 + pair_index,
                )
                paired_comparisons.append({
                    "first_id": first_id,
                    "second_id": other_id,
                    "metric": metric,
                    "common_epoch_count": len(values),
                    "difference": describe(values),
                    "sign_test_pvalue": paired_sign_test_pvalue(values),
                    "bootstrap_confidence": bootstrap_confidence,
                    "bootstrap_mean_ci_low": low,
                    "bootstrap_mean_ci_high": high,
                })

    residual_groups = _residual_groups(residual_rows)
    output_directory.mkdir(parents=True, exist_ok=True)
    write_csv(output_directory / "ppp_epochs.csv", rows)
    write_csv(output_directory / "ppp_common_epoch_differences.csv", paired_rows)
    write_csv(output_directory / "ppp_jump_events.csv", jump_rows)
    write_csv(output_directory / "ppp_fixed_segments.csv", segment_rows)
    write_csv(output_directory / "ppp_reconvergence_events.csv", reconvergence_rows)
    write_csv(output_directory / "ppp_residuals.csv", residual_rows)
    write_csv(output_directory / "ppp_residual_summary.csv", residual_groups)
    summary = {
        "datasets": summaries,
        "common_epoch_count": common_epoch_count,
        "common_epoch_coverage": common_epoch_coverage,
        "paired_comparisons": paired_comparisons,
        "residual_groups": residual_groups,
        "convergence_definition": {
            "horizontal_threshold_m": horizontal_threshold_m,
            "vertical_threshold_m": vertical_threshold_m,
            "sustain_seconds": sustain_seconds,
        },
        "warnings": warnings,
    }
    validate_metrics("ppp", json_value(summary))
    write_json(output_directory / "metrics.json", summary)
    figures = [
        ppp_figure(rows, output_directory / "ppp_enu_fixing.png"),
        ppp_error_histogram_figure(rows, output_directory / "ppp_error_histograms.png"),
        ppp_ztd_figure(rows, output_directory / "ppp_ztd_diagnostics.png"),
        ppp_residual_figure(
            residual_rows, output_directory / "ppp_residual_diagnostics.png"
        ),
    ]
    write_html_report(
        output_directory / "report.html",
        "PPP Result Analysis",
        summary,
        [figure for figure in figures if figure],
        warnings,
    )
    write_manifest(output_directory, "ppp", config_path, config, input_paths, warnings)
    return summary
