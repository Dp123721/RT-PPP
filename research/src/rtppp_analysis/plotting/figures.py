"""Creates non-interactive figures for orbit, observation, and PPP reports."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def _finish(fig: Any, path: Path) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return path


def orbit_figure(rows: list[dict[str, Any]], path: Path) -> Path | None:
    rows = [row for row in rows if row.get("comparison_scope") == "reference"]
    if not rows:
        return None
    fig, axes = plt.subplots(5, 1, figsize=(12, 12), sharex=True)
    fields = [
        ("radial_m", "Radial (m)"),
        ("along_m", "Along (m)"),
        ("cross_m", "Cross (m)"),
        ("clock_datum_removed_m", "Clock, datum removed (m)"),
        ("sisre_m", "SISRE (m)"),
    ]
    for candidate in sorted({str(row["candidate_id"]) for row in rows}):
        selected = [row for row in rows if row["candidate_id"] == candidate]
        x = np.asarray([(row["epoch_ns"] - selected[0]["epoch_ns"]) / 3600e9 for row in selected])
        for axis, (field, label) in zip(axes, fields):
            y = np.asarray([row.get(field, np.nan) for row in selected], dtype=float)
            axis.plot(x, y, ".", markersize=2, label=candidate)
            axis.set_ylabel(label)
            axis.grid(True, alpha=0.25)
    axes[0].legend(loc="best")
    axes[-1].set_xlabel("Hours from first sample")
    return _finish(fig, path)


def _finite_values(rows: list[dict[str, Any]], field: str) -> np.ndarray:
    values = np.asarray([row.get(field, np.nan) for row in rows], dtype=float)
    return values[np.isfinite(values)]


def orbit_histogram_figure(rows: list[dict[str, Any]], path: Path) -> Path | None:
    rows = [row for row in rows if row.get("comparison_scope") == "reference"]
    if not rows:
        return None
    fields = [
        ("radial_m", "Radial residual (m)"),
        ("along_m", "Along-track residual (m)"),
        ("cross_m", "Cross-track residual (m)"),
        ("orbit_3d_m", "3D orbit error (m)"),
        ("clock_raw_m", "Raw clock residual (m)"),
        ("clock_datum_removed_m", "Datum-removed clock residual (m)"),
        ("sisre_m", "SISRE (m)"),
    ]
    candidates = sorted({str(row["candidate_id"]) for row in rows})
    comparisons = sorted(
        {f"{row['candidate_id']} vs {row['reference_id']}" for row in rows}
    )
    fig, axes = plt.subplots(3, 3, figsize=(15, 11))
    fig.suptitle(f"Orbit and clock residual distributions: {', '.join(comparisons)}")
    for axis, (field, label) in zip(axes.flat, fields):
        all_values = _finite_values(rows, field)
        if all_values.size:
            value_min = float(np.min(all_values))
            value_max = float(np.max(all_values))
            bins: int | np.ndarray = 50
            if value_min == value_max:
                padding = max(abs(value_min) * 0.05, 1e-12)
                bins = np.linspace(value_min - padding, value_max + padding, 3)
            for candidate in candidates:
                selected = [row for row in rows if row["candidate_id"] == candidate]
                values = _finite_values(selected, field)
                if values.size:
                    axis.hist(
                        values,
                        bins=bins,
                        range=None if isinstance(bins, np.ndarray) else (value_min, value_max),
                        histtype="step",
                        linewidth=1.4,
                        label=f"{candidate} (n={values.size})",
                    )
        if field in {"radial_m", "along_m", "cross_m", "clock_datum_removed_m"}:
            axis.axvline(0.0, color="#374151", linestyle="--", linewidth=0.9)
        axis.set_xlabel(label)
        axis.set_ylabel("Sample count")
        axis.grid(True, axis="y", alpha=0.25)
        if len(candidates) > 1:
            axis.legend(loc="best", fontsize="small")
    for axis in axes.flat[len(fields) :]:
        axis.set_visible(False)
    return _finish(fig, path)


def orbit_satellite_rms_figure(rows: list[dict[str, Any]], path: Path) -> Path | None:
    rows = [row for row in rows if row.get("comparison_scope") == "reference"]
    if not rows:
        return None
    groups: dict[tuple[str, str], list[dict[str, Any]]] = {}
    for row in rows:
        groups.setdefault((str(row["candidate_id"]), str(row["satellite"])), []).append(row)
    labels = [
        satellite if len({key[0] for key in groups}) == 1 else f"{candidate}:{satellite}"
        for candidate, satellite in sorted(groups)
    ]
    selected_groups = [groups[key] for key in sorted(groups)]

    def rms(selected: list[dict[str, Any]], field: str) -> float:
        values = _finite_values(selected, field)
        return float(np.sqrt(np.mean(values * values))) if values.size else float("nan")

    fig_width = max(12.0, min(24.0, 0.42 * len(labels)))
    fig, axes = plt.subplots(3, 1, figsize=(fig_width, 10), sharex=True)
    comparisons = sorted(
        {f"{row['candidate_id']} vs {row['reference_id']}" for row in rows}
    )
    fig.suptitle(f"Per-satellite RMS (lower is better): {', '.join(comparisons)}")
    x = np.arange(len(labels), dtype=float)
    orbit_fields = (
        ("radial_m", "Radial"),
        ("along_m", "Along"),
        ("cross_m", "Cross"),
        ("orbit_3d_m", "3D"),
    )
    width = 0.2
    for index, (field, label) in enumerate(orbit_fields):
        values = [rms(selected, field) for selected in selected_groups]
        axes[0].bar(x + (index - 1.5) * width, values, width=width, label=label)
    axes[0].set_ylabel("Orbit RMS (m)")
    axes[0].grid(True, axis="y", alpha=0.25)
    axes[0].legend(loc="best", ncol=4)

    clock_values = [rms(selected, "clock_datum_removed_m") for selected in selected_groups]
    axes[1].bar(x, clock_values, width=0.7, color="#4472c4")
    axes[1].set_ylabel("Datum-removed clock RMS (m)")
    axes[1].grid(True, axis="y", alpha=0.25)
    sisre_values = [rms(selected, "sisre_m") for selected in selected_groups]
    axes[2].bar(x, sisre_values, width=0.7, color="#2f855a")
    axes[2].set_ylabel("SISRE RMS (m)")
    axes[2].set_xlabel("Satellite")
    axes[2].grid(True, axis="y", alpha=0.25)
    axes[2].set_xticks(x, labels, rotation=60, ha="right")
    return _finish(fig, path)


def observation_figure(
    epoch_rows: list[dict[str, Any]], snr_rows: list[dict[str, Any]], path: Path
) -> Path | None:
    if not epoch_rows:
        return None
    fig, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=False)
    for dataset in sorted({str(row["dataset_id"]) for row in epoch_rows}):
        selected = [row for row in epoch_rows if row["dataset_id"] == dataset]
        x = np.asarray([(row["epoch_ns"] - selected[0]["epoch_ns"]) / 3600e9 for row in selected])
        axes[0].plot(x, [row["satellite_count"] for row in selected], label=dataset)
    axes[0].set_ylabel("Observed satellites")
    axes[0].set_xlabel("Hours from first sample")
    axes[0].grid(True, alpha=0.25)
    axes[0].legend(loc="best")
    if snr_rows:
        labels = sorted({f"{row['system']}:{row['observation_type']}" for row in snr_rows})
        values = [
            np.nanmedian(
                [
                    row["snr_dbhz"]
                    for row in snr_rows
                    if f"{row['system']}:{row['observation_type']}" == label
                ]
            )
            for label in labels
        ]
        axes[1].bar(np.arange(len(labels)), values)
        axes[1].set_xticks(np.arange(len(labels)), labels, rotation=45, ha="right")
    axes[1].set_ylabel("Median SNR (dB-Hz)")
    axes[1].grid(True, axis="y", alpha=0.25)
    return _finish(fig, path)


def observation_signal_coverage_figure(
    signal_rows: list[dict[str, Any]], path: Path
) -> Path | None:
    if not signal_rows:
        return None
    selected = sorted(
        signal_rows,
        key=lambda row: (
            str(row["dataset_id"]),
            str(row["system"]),
            str(row["observation_type"]),
        ),
    )
    labels = [
        f"{row['dataset_id']}:{row['system']}:{row['observation_type']}" for row in selected
    ]
    y = np.arange(len(labels), dtype=float)
    fig, axis = plt.subplots(figsize=(12, max(6.0, 0.3 * len(labels))))
    width = 0.24
    for offset, field, label in (
        (-width, "value_coverage_rate", "Value opportunities"),
        (0.0, "epoch_coverage_rate", "Epochs"),
        (width, "satellite_coverage_rate", "Satellites"),
    ):
        axis.barh(
            y + offset,
            [row.get(field, np.nan) for row in selected],
            height=width,
            label=label,
        )
    axis.set_yticks(y, labels)
    axis.set_xlim(0.0, 1.0)
    axis.set_xlabel("Coverage rate")
    axis.set_title("Observation signal coverage", pad=36)
    axis.grid(True, axis="x", alpha=0.25)
    axis.legend(
        loc="lower center", bbox_to_anchor=(0.5, 1.01), ncol=3, frameon=False
    )
    axis.invert_yaxis()
    return _finish(fig, path)


def observation_snr_histogram_figure(
    snr_rows: list[dict[str, Any]], path: Path
) -> Path | None:
    if not snr_rows:
        return None
    groups: dict[tuple[str, str, str], list[float]] = {}
    for row in snr_rows:
        groups.setdefault(
            (str(row["dataset_id"]), str(row["system"]), str(row["observation_type"])),
            [],
        ).append(float(row["snr_dbhz"]))
    all_values = np.asarray([value for values in groups.values() for value in values], dtype=float)
    all_values = all_values[np.isfinite(all_values)]
    if not all_values.size:
        return None
    value_min = float(np.min(all_values))
    value_max = float(np.max(all_values))
    if value_min == value_max:
        padding = max(abs(value_min) * 0.05, 0.5)
        bins = np.linspace(value_min - padding, value_max + padding, 3)
    else:
        bin_count = min(40, max(5, int(np.sqrt(all_values.size))))
        bins = np.linspace(value_min, value_max, bin_count + 1)
    fig, axis = plt.subplots(figsize=(12, 6))
    for (dataset, system, observation_type), values in sorted(groups.items()):
        finite = np.asarray(values, dtype=float)
        finite = finite[np.isfinite(finite)]
        if finite.size:
            axis.hist(
                finite,
                bins=bins,
                histtype="step",
                linewidth=1.4,
                label=f"{dataset}:{system}:{observation_type} (n={finite.size})",
            )
    axis.set_xlabel("SNR (dB-Hz)")
    axis.set_ylabel("Sample count")
    axis.set_title("SNR distributions")
    axis.grid(True, axis="y", alpha=0.25)
    axis.legend(loc="best", fontsize="small")
    return _finish(fig, path)


def observation_elevation_snr_figure(
    rows: list[dict[str, Any]], path: Path
) -> Path | None:
    if not rows:
        return None
    groups: dict[tuple[str, str, str, int], list[tuple[float, int]]] = {}
    for row in rows:
        mean = float(row.get("snr_mean", np.nan))
        count = int(row.get("snr_count", 0))
        if not np.isfinite(mean) or count <= 0:
            continue
        key = (
            str(row["dataset_id"]), str(row["source_id"]), str(row["signal"]),
            int(row["elevation_min_deg"]),
        )
        groups.setdefault(key, []).append((mean, count))
    if not groups:
        return None
    series: dict[tuple[str, str, str], list[tuple[float, float, int]]] = {}
    for (dataset, source, signal, lower), values in sorted(groups.items()):
        total = sum(count for _, count in values)
        weighted_mean = sum(mean * count for mean, count in values) / total
        series.setdefault((dataset, source, signal), []).append(
            (lower + 5.0, weighted_mean, total)
        )
    fig, axis = plt.subplots(figsize=(12, 6))
    for (dataset, source, signal), values in sorted(series.items()):
        axis.plot(
            [value[0] for value in values], [value[1] for value in values],
            marker="o", linewidth=1.2, label=f"{dataset}:{source}:{signal}",
        )
    axis.set_xlim(0.0, 90.0)
    axis.set_xticks(np.arange(0.0, 91.0, 10.0))
    axis.set_xlabel("Elevation bin midpoint (degrees)")
    axis.set_ylabel("Mean CNR (dB-Hz)")
    axis.set_title("Elevation-binned signal strength")
    axis.grid(True, alpha=0.25)
    axis.legend(loc="best", fontsize="small")
    return _finish(fig, path)


def observation_tracking_figure(
    arc_rows: list[dict[str, Any]], slip_rows: list[dict[str, Any]], path: Path
) -> Path | None:
    if not arc_rows and not slip_rows:
        return None
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    durations = _finite_values(arc_rows, "duration_seconds")
    if durations.size:
        bin_count = min(40, max(5, int(np.sqrt(durations.size))))
        axes[0].hist(durations, bins=bin_count, color="#4472c4", edgecolor="white")
    axes[0].set_xlabel("Continuous tracking duration (s)")
    axes[0].set_ylabel("Arc count")
    axes[0].set_title("Lock arc durations")
    axes[0].grid(True, axis="y", alpha=0.25)
    evidence_counts: dict[str, int] = {}
    for row in slip_rows:
        evidence = str(row.get("evidence", "unknown")) or "unknown"
        evidence_counts[evidence] = evidence_counts.get(evidence, 0) + 1
    labels = sorted(evidence_counts)
    axes[1].bar(labels, [evidence_counts[label] for label in labels], color="#c44e52")
    axes[1].set_ylabel("Detected event count")
    axes[1].set_title("Cycle-slip evidence")
    axes[1].grid(True, axis="y", alpha=0.25)
    axes[1].tick_params(axis="x", rotation=30)
    return _finish(fig, path)


def observation_reception_figure(
    reception_rows: list[dict[str, Any]], ssr_rows: list[dict[str, Any]], path: Path
) -> Path | None:
    if not reception_rows and not ssr_rows:
        return None
    status_counts: dict[str, int] = {}
    reason_counts: dict[str, int] = {}
    for row in reception_rows:
        status = str(row.get("status", "unknown")) or "unknown"
        status_counts[status] = status_counts.get(status, 0) + 1
        reason = str(row.get("reason", ""))
        if reason:
            reason_counts[reason] = reason_counts.get(reason, 0) + 1
    for row in ssr_rows:
        reason = str(row.get("reason", ""))
        if reason:
            reason_counts[reason] = reason_counts.get(reason, 0) + 1
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    statuses = sorted(status_counts)
    axes[0].bar(statuses, [status_counts[value] for value in statuses], color="#4c956c")
    axes[0].set_title("Reception event status")
    axes[0].set_ylabel("Event count")
    axes[0].grid(True, axis="y", alpha=0.25)
    axes[0].tick_params(axis="x", rotation=30)
    reasons = sorted(reason_counts)
    axes[1].bar(reasons, [reason_counts[value] for value in reasons], color="#dd8452")
    axes[1].set_title("Evidence-backed rejection/interruption reasons")
    axes[1].set_ylabel("Event count")
    axes[1].grid(True, axis="y", alpha=0.25)
    axes[1].tick_params(axis="x", rotation=30)
    return _finish(fig, path)


def ppp_figure(rows: list[dict[str, Any]], path: Path) -> Path | None:
    if not rows:
        return None
    fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
    for dataset in sorted({str(row["dataset_id"]) for row in rows}):
        selected = [row for row in rows if row["dataset_id"] == dataset]
        x = np.asarray([(row["epoch_ns"] - selected[0]["epoch_ns"]) / 3600e9 for row in selected])
        for axis, field, label in zip(
            axes[:3], ("north_m", "east_m", "up_m"), ("North (m)", "East (m)", "Up (m)")
        ):
            axis.plot(x, [row[field] for row in selected], linewidth=0.8, label=dataset)
            axis.set_ylabel(label)
            axis.grid(True, alpha=0.25)
        axes[3].plot(x, [row["fixed_count"] for row in selected], label=dataset)
    axes[0].legend(loc="best")
    axes[3].set_ylabel("Fixed satellites")
    axes[3].set_xlabel("Hours from first sample")
    axes[3].grid(True, alpha=0.25)
    return _finish(fig, path)


def ppp_error_histogram_figure(
    rows: list[dict[str, Any]], path: Path
) -> Path | None:
    if not rows:
        return None
    fields = (
        ("north_m", "North error (m)"),
        ("east_m", "East error (m)"),
        ("up_m", "Up error (m)"),
        ("horizontal_m", "Horizontal error (m)"),
        ("error_3d_m", "3D error (m)"),
        ("nees", "Position NEES"),
    )
    fig, axes = plt.subplots(2, 3, figsize=(15, 8))
    for axis, (field, label) in zip(axes.flat, fields):
        for dataset in sorted({str(row["dataset_id"]) for row in rows}):
            values = _finite_values(
                [row for row in rows if row["dataset_id"] == dataset], field
            )
            if values.size:
                axis.hist(values, bins=min(40, max(5, int(np.sqrt(values.size)))),
                          histtype="step", linewidth=1.4, label=dataset)
        axis.set_xlabel(label)
        axis.set_ylabel("Epoch count")
        axis.grid(True, axis="y", alpha=0.25)
    axes[0, 0].legend(loc="best", fontsize="small")
    return _finish(fig, path)


def ppp_ztd_figure(rows: list[dict[str, Any]], path: Path) -> Path | None:
    if not rows:
        return None
    fig, axes = plt.subplots(4, 1, figsize=(12, 9), sharex=True)
    fields = (
        ("ztd_m", "ZTD (m)"),
        ("wet_residual_m", "Wet delay residual (m)"),
        ("ztd_std_m", "ZTD standard deviation (m)"),
        ("hdop", "HDOP"),
    )
    for dataset in sorted({str(row["dataset_id"]) for row in rows}):
        selected = [row for row in rows if row["dataset_id"] == dataset]
        x = np.asarray([
            (row["epoch_ns"] - selected[0]["epoch_ns"]) / 3600e9 for row in selected
        ])
        for axis, (field, label) in zip(axes, fields):
            axis.plot(x, [row[field] for row in selected], linewidth=0.9, label=dataset)
            axis.set_ylabel(label)
            axis.grid(True, alpha=0.25)
    axes[0].legend(loc="best")
    axes[-1].set_xlabel("Hours from first sample")
    return _finish(fig, path)


def ppp_residual_figure(
    rows: list[dict[str, Any]], path: Path
) -> Path | None:
    if not rows:
        return None
    adopted = [row for row in rows if row.get("adopted")]
    fig, axes = plt.subplots(2, 2, figsize=(14, 9))
    for key in sorted({
        (str(row["dataset_id"]), str(row["observation_type"]), str(row["signal"]))
        for row in adopted
    }):
        selected = [
            row for row in adopted
            if (row["dataset_id"], row["observation_type"], row["signal"]) == key
        ]
        if not selected:
            continue
        first = min(row["epoch_ns"] for row in selected)
        label = ":".join(key)
        axes[0, 0].plot(
            [(row["epoch_ns"] - first) / 3600e9 for row in selected],
            [row["postfit_m"] for row in selected], ".", markersize=3, label=label,
        )
        axes[0, 1].hist(
            [row["normalized_postfit"] for row in selected], bins=30,
            histtype="step", linewidth=1.3, label=label,
        )
        axes[1, 0].scatter(
            [row["elevation_deg"] for row in selected],
            [abs(row["postfit_m"]) for row in selected], s=10, alpha=0.65, label=label,
        )
    axes[0, 0].axhline(0.0, color="#374151", linestyle="--", linewidth=0.8)
    axes[0, 0].set_xlabel("Hours from first residual")
    axes[0, 0].set_ylabel("Postfit residual (m)")
    axes[0, 0].set_title("Residual time series")
    axes[0, 1].set_xlabel("Normalized postfit residual")
    axes[0, 1].set_ylabel("Observation count")
    axes[0, 1].set_title("Normalized residual distribution")
    axes[1, 0].set_xlabel("Elevation (degrees)")
    axes[1, 0].set_ylabel("Absolute postfit residual (m)")
    axes[1, 0].set_title("Elevation dependence")
    reason_counts: dict[str, int] = {}
    for row in rows:
        reason = "adopted" if row.get("adopted") else str(row.get("reason") or "unknown")
        reason_counts[reason] = reason_counts.get(reason, 0) + 1
    labels = sorted(reason_counts)
    axes[1, 1].bar(labels, [reason_counts[label] for label in labels], color="#dd8452")
    axes[1, 1].set_ylabel("Observation count")
    axes[1, 1].set_title("Adoption and rejection reasons")
    axes[1, 1].tick_params(axis="x", rotation=30)
    for axis in axes.flat:
        axis.grid(True, alpha=0.25)
    if adopted:
        axes[0, 0].legend(loc="best", fontsize="small")
    return _finish(fig, path)
