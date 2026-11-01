"""Runs observation coverage and signal-quality analysis."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np

from ..core.statistics import describe
from ..core.time import unix_ns_to_datetime
from ..contracts import validate_metrics
from ..io.rinex_obs import read_rinex_observation_file
from ..io.tables import json_value, write_csv, write_json
from ..models import MissingReason
from ..plotting.figures import (
    observation_elevation_snr_figure,
    observation_figure,
    observation_reception_figure,
    observation_signal_coverage_figure,
    observation_snr_histogram_figure,
    observation_tracking_figure,
)
from ..provenance import write_manifest
from ..reports.html import write_html_report
from .telemetry import (
    analyze_tracking,
    elevation_snr_summary,
    read_telemetry,
    write_partitioned_parquet,
)


def _rate(count: int, total: int) -> float:
    return count / total if total else float("nan")


def _stats_columns(prefix: str, values: Any) -> dict[str, Any]:
    return {f"{prefix}_{key}": value for key, value in describe(values).items()}


def run_observation_analysis(
    config: dict[str, Any], config_path: Path, output_directory: Path
) -> dict[str, Any]:
    datasets = config.get("datasets", [])
    if not isinstance(datasets, list) or not datasets:
        raise ValueError("Observation analysis requires at least one dataset")
    options = config.get("options", {})
    expected_interval_s = float(options.get("expected_interval_seconds", 0.0))
    gap_factor = float(options.get("gap_factor", 1.5))
    epoch_rows: list[dict[str, Any]] = []
    observation_rows: list[dict[str, Any]] = []
    snr_rows: list[dict[str, Any]] = []
    signal_rows: list[dict[str, Any]] = []
    snr_satellite_rows: list[dict[str, Any]] = []
    gap_rows: list[dict[str, Any]] = []
    event_rows: list[dict[str, Any]] = []
    msm_telemetry_rows: list[dict[str, Any]] = []
    ssr_telemetry_rows: list[dict[str, Any]] = []
    reception_rows: list[dict[str, Any]] = []
    lock_arc_rows: list[dict[str, Any]] = []
    cycle_slip_rows: list[dict[str, Any]] = []
    half_cycle_rows: list[dict[str, Any]] = []
    snr_elevation_rows: list[dict[str, Any]] = []
    summaries: list[dict[str, Any]] = []
    input_paths: list[Path] = []
    warnings: list[str] = []

    for dataset in datasets:
        dataset_id = str(dataset.get("id", "")).strip()
        station = str(dataset.get("station", "")).strip()
        if not dataset_id:
            raise ValueError("Every observation dataset requires an id")
        rinex_path = Path(str(dataset.get("rinex", ""))).expanduser()
        if not rinex_path.is_absolute():
            rinex_path = config_path.parent / rinex_path
        rinex_path = rinex_path.resolve()
        input_paths.append(rinex_path)
        time_scale = str(dataset.get("time_scale", "UTC")).upper()
        parsed = read_rinex_observation_file(rinex_path, time_scale)
        observations = parsed.observations

        by_epoch: dict[int, list] = {epoch: [] for epoch in parsed.epoch_satellites}
        by_signal: dict[tuple[str, str], list] = {}
        for observation in observations:
            by_epoch[observation.epoch_ns].append(observation)
            by_signal.setdefault(
                (observation.system, observation.observation_type), []
            ).append(observation)
            row = {
                "dataset_id": dataset_id,
                "station": station,
                "epoch_ns": observation.epoch_ns,
                "epoch_utc": unix_ns_to_datetime(observation.epoch_ns).isoformat(),
                "satellite": observation.satellite,
                "system": observation.system,
                "observation_type": observation.observation_type,
                "signal": observation.signal,
                "value": observation.value,
                "lli": observation.lli,
                "ssi": observation.ssi,
            }
            observation_rows.append(row)
            if observation.observation_type.startswith("S"):
                snr_rows.append({**row, "snr_dbhz": observation.value})

        epochs = np.asarray(sorted(parsed.epoch_satellites), dtype=np.int64)
        all_satellites = {
            satellite
            for satellites in parsed.epoch_satellites.values()
            for satellite in satellites
        }
        reported_systems = sorted({*parsed.observation_types, "G", "R", "E", "C"})
        system_satellites = {
            system: {satellite for satellite in all_satellites if satellite.startswith(system)}
            for system in reported_systems
        }
        system_record_counts = {
            system: sum(
                satellite.startswith(system)
                for satellites in parsed.epoch_satellites.values()
                for satellite in satellites
            )
            for system in reported_systems
        }

        inferred_interval_s = float("nan")
        if epochs.size >= 2:
            inferred_interval_s = float(np.median(np.diff(epochs)) / 1e9)
        interval_s = expected_interval_s if expected_interval_s > 0.0 else inferred_interval_s
        maximum_gap_s = float("nan")
        if epochs.size >= 2:
            gaps_s = np.diff(epochs) / 1e9
            maximum_gap_s = float(np.max(gaps_s))
            if np.isfinite(interval_s) and interval_s > 0.0:
                for before, after, gap_s in zip(epochs[:-1], epochs[1:], gaps_s):
                    if gap_s <= interval_s * gap_factor:
                        continue
                    estimated_missing = max(0, int(round(gap_s / interval_s)) - 1)
                    gap_rows.append(
                        {
                            "dataset_id": dataset_id,
                            "station": station,
                            "previous_epoch_ns": int(before),
                            "previous_epoch_utc": unix_ns_to_datetime(int(before)).isoformat(),
                            "next_epoch_ns": int(after),
                            "next_epoch_utc": unix_ns_to_datetime(int(after)).isoformat(),
                            "gap_seconds": float(gap_s),
                            "expected_interval_seconds": interval_s,
                            "estimated_missing_epoch_count": estimated_missing,
                        }
                    )

        dataset_gap_rows = [row for row in gap_rows if row["dataset_id"] == dataset_id]

        dataset_msm: list[dict[str, Any]] = []
        dataset_ssr: list[dict[str, Any]] = []
        dataset_reception: list[dict[str, Any]] = []
        reception_status_counts: dict[str, int] = {}
        reception_reason_counts: dict[str, int] = {}
        dataset_arcs: list[dict[str, Any]] = []
        dataset_slips: list[dict[str, Any]] = []
        dataset_half_cycles: list[dict[str, Any]] = []
        dataset_elevation: list[dict[str, Any]] = []
        parquet_paths: list[Path] = []
        telemetry_value = str(dataset.get("telemetry_directory", "")).strip()
        if telemetry_value:
            telemetry_directory = Path(telemetry_value).expanduser()
            if not telemetry_directory.is_absolute():
                telemetry_directory = config_path.parent / telemetry_directory
            telemetry_directory = telemetry_directory.resolve()
            source_id = str(dataset.get("telemetry_source_id", "")).strip() or None
            telemetry = read_telemetry(telemetry_directory, source_id)
            for family in ("msm", "ssr", "reception"):
                input_paths.extend(sorted(telemetry_directory.glob(f"{family}_*.csv")))
            dataset_msm = [{"dataset_id": dataset_id, "station": station, **row}
                           for row in telemetry["msm"]]
            dataset_ssr = [{"dataset_id": dataset_id, "station": station, **row}
                           for row in telemetry["ssr"]]
            dataset_reception = [{"dataset_id": dataset_id, "station": station, **row}
                                 for row in telemetry["reception"]]
            for row in dataset_reception:
                status = str(row.get("status", "")).strip() or "unknown"
                reception_status_counts[status] = reception_status_counts.get(status, 0) + 1
                reason = str(row.get("reason", "")).strip()
                if reason:
                    reception_reason_counts[reason] = reception_reason_counts.get(reason, 0) + 1
            analysis_interval = interval_s if np.isfinite(interval_s) and interval_s > 0 else 1.0
            arcs, slips, half_cycles = analyze_tracking(
                dataset_msm, float(analysis_interval), gap_factor
            )
            dataset_arcs = [{"dataset_id": dataset_id, "station": station, **row} for row in arcs]
            dataset_slips = [{"dataset_id": dataset_id, "station": station, **row} for row in slips]
            dataset_half_cycles = [
                {"dataset_id": dataset_id, "station": station, **row} for row in half_cycles
            ]
            dataset_elevation = [
                {"dataset_id": dataset_id, "station": station, **row}
                for row in elevation_snr_summary(dataset_msm)
            ]
            if bool(options.get("write_parquet", False)):
                parquet_paths = write_partitioned_parquet(
                    output_directory / "telemetry_parquet",
                    station,
                    {"msm": dataset_msm, "ssr": dataset_ssr, "reception": dataset_reception},
                )
            msm_telemetry_rows.extend(dataset_msm)
            ssr_telemetry_rows.extend(dataset_ssr)
            reception_rows.extend(dataset_reception)
            lock_arc_rows.extend(dataset_arcs)
            cycle_slip_rows.extend(dataset_slips)
            half_cycle_rows.extend(dataset_half_cycles)
            snr_elevation_rows.extend(dataset_elevation)
        else:
            warnings.append(
                f"{dataset_id}: structured MSM/SSR telemetry is unavailable; absent messages "
                "remain unknown and are not classified as packet loss."
            )
            warnings.append(
                f"{dataset_id}: satellite elevation telemetry is unavailable; elevation-binned "
                "SNR statistics were not generated."
            )
        for epoch_ns in epochs:
            values = by_epoch[int(epoch_ns)]
            satellites = parsed.epoch_satellites[int(epoch_ns)]
            epoch_rows.append(
                {
                    "dataset_id": dataset_id,
                    "station": station,
                    "epoch_ns": int(epoch_ns),
                    "epoch_utc": unix_ns_to_datetime(int(epoch_ns)).isoformat(),
                    "satellite_count": len(satellites),
                    "gps_count": len({sat for sat in satellites if sat.startswith("G")}),
                    "glonass_count": len({sat for sat in satellites if sat.startswith("R")}),
                    "galileo_count": len({sat for sat in satellites if sat.startswith("E")}),
                    "bds_count": len({sat for sat in satellites if sat.startswith("C")}),
                    "other_count": len(
                        {sat for sat in satellites if sat[:1] not in {"G", "R", "E", "C"}}
                    ),
                    "observation_value_count": len(values),
                    "snr_value_count": sum(
                        value.observation_type.startswith("S") for value in values
                    ),
                }
            )

        for epoch_ns, flag, record_count in parsed.event_epochs:
            event_rows.append(
                {
                    "dataset_id": dataset_id,
                    "station": station,
                    "epoch_ns": epoch_ns,
                    "epoch_utc": unix_ns_to_datetime(epoch_ns).isoformat(),
                    "event_flag": flag,
                    "special_record_count": record_count,
                }
            )

        for system, observation_types in sorted(parsed.observation_types.items()):
            possible_count = system_record_counts[system]
            system_satellite_count = len(system_satellites[system])
            for observation_type in observation_types:
                values = by_signal.get((system, observation_type), [])
                signal_epoch_count = len({value.epoch_ns for value in values})
                signal_satellite_count = len({value.satellite for value in values})
                row = {
                    "dataset_id": dataset_id,
                    "station": station,
                    "system": system,
                    "observation_type": observation_type,
                    "signal": observation_type[1:],
                    "value_count": len(values),
                    "possible_value_count": possible_count,
                    "missing_value_count": possible_count - len(values),
                    "value_coverage_rate": _rate(len(values), possible_count),
                    "epoch_count": signal_epoch_count,
                    "total_epoch_count": int(epochs.size),
                    "satellite_count": signal_satellite_count,
                    "total_system_satellite_count": system_satellite_count,
                    "epoch_coverage_rate": _rate(signal_epoch_count, int(epochs.size)),
                    "satellite_coverage_rate": _rate(
                        signal_satellite_count, system_satellite_count
                    ),
                    "lli_value_count": sum(value.lli is not None for value in values),
                    "ssi_value_count": sum(value.ssi is not None for value in values),
                }
                if observation_type.startswith("S"):
                    row.update(_stats_columns("snr", (item.value for item in values)))
                signal_rows.append(row)

        snr_groups: dict[tuple[str, str, str], list] = {}
        for observation in observations:
            if observation.observation_type.startswith("S"):
                snr_groups.setdefault(
                    (observation.system, observation.observation_type, observation.satellite), []
                ).append(observation)
        for (system, observation_type, satellite), values in sorted(snr_groups.items()):
            snr_satellite_rows.append(
                {
                    "dataset_id": dataset_id,
                    "station": station,
                    "system": system,
                    "satellite": satellite,
                    "observation_type": observation_type,
                    "signal": observation_type[1:],
                    **_stats_columns("snr", (item.value for item in values)),
                }
            )

        longest_interruption_s = float("nan")
        if dataset_gap_rows and np.isfinite(interval_s):
            longest_interruption_s = max(
                float(row["gap_seconds"]) - interval_s for row in dataset_gap_rows
            )
        summaries.append(
            {
                "dataset_id": dataset_id,
                "station": station,
                "epoch_count": int(epochs.size),
                "event_epoch_count": len(parsed.event_epochs),
                "satellite_record_count": sum(
                    len(satellites) for satellites in parsed.epoch_satellites.values()
                ),
                "observation_value_count": len(observations),
                "first_epoch_utc": (
                    unix_ns_to_datetime(int(epochs[0])).isoformat() if epochs.size else None
                ),
                "last_epoch_utc": (
                    unix_ns_to_datetime(int(epochs[-1])).isoformat() if epochs.size else None
                ),
                "duration_seconds": (
                    float((epochs[-1] - epochs[0]) / 1e9) if epochs.size >= 2 else 0.0
                ),
                "inferred_interval_seconds": inferred_interval_s,
                "gap_count": len(dataset_gap_rows),
                "estimated_missing_epoch_count": sum(
                    int(row["estimated_missing_epoch_count"]) for row in dataset_gap_rows
                ),
                "maximum_gap_seconds": maximum_gap_s,
                "longest_interruption_seconds": longest_interruption_s,
                "unique_satellite_count": len(all_satellites),
                "system_satellite_counts": {
                    system: len(satellites)
                    for system, satellites in sorted(system_satellites.items())
                },
                "satellite_count_stats": describe(
                    len(parsed.epoch_satellites[int(epoch)]) for epoch in epochs
                ),
                "telemetry_analysis_available": bool(dataset_msm),
                "telemetry_unavailable_reason": (
                    None if dataset_msm else MissingReason.UNKNOWN.value
                ),
                "msm_telemetry_count": len(dataset_msm),
                "ssr_telemetry_count": len(dataset_ssr),
                "reception_event_count": len(dataset_reception),
                "reception_status_counts": reception_status_counts,
                "reception_reason_counts": reception_reason_counts,
                "stream_interruption_count": reception_reason_counts.get("stream_interruption", 0),
                "lock_arc_count": len(dataset_arcs),
                "cycle_slip_count": len(dataset_slips),
                "half_cycle_event_count": len(dataset_half_cycles),
                "maximum_continuous_tracking_seconds": (
                    max((float(row["duration_seconds"]) for row in dataset_arcs), default=float("nan"))
                ),
                "ssr_reception_analysis_available": bool(dataset_ssr or dataset_reception),
                "ssr_reception_unavailable_reason": (
                    None if (dataset_ssr or dataset_reception) else MissingReason.UNKNOWN.value
                ),
                "snr_elevation_analysis_available": bool(dataset_elevation),
                "snr_elevation_unavailable_reason": (
                    None if dataset_elevation else MissingReason.UNKNOWN.value
                ),
                "parquet_partition_count": len(parquet_paths),
            }
        )

    output_directory.mkdir(parents=True, exist_ok=True)
    write_csv(output_directory / "observation_epochs.csv", epoch_rows)
    write_csv(output_directory / "observation_values.csv", observation_rows)
    write_csv(output_directory / "observation_events.csv", event_rows)
    write_csv(output_directory / "observation_gaps.csv", gap_rows)
    write_csv(output_directory / "snr_values.csv", snr_rows)
    write_csv(output_directory / "signal_summary.csv", signal_rows)
    write_csv(output_directory / "snr_satellite_summary.csv", snr_satellite_rows)
    write_csv(output_directory / "msm_telemetry.csv", msm_telemetry_rows)
    write_csv(output_directory / "ssr_telemetry.csv", ssr_telemetry_rows)
    write_csv(output_directory / "reception_events.csv", reception_rows)
    write_csv(output_directory / "lock_arcs.csv", lock_arc_rows)
    write_csv(output_directory / "cycle_slips.csv", cycle_slip_rows)
    write_csv(output_directory / "half_cycle_events.csv", half_cycle_rows)
    write_csv(output_directory / "snr_elevation_summary.csv", snr_elevation_rows)
    summary = {
        "datasets": summaries,
        "signal_groups": signal_rows,
        "snr_satellite_groups": snr_satellite_rows,
        "snr_elevation_groups": snr_elevation_rows,
        "warnings": warnings,
    }
    validate_metrics("observations", json_value(summary))
    write_json(output_directory / "metrics.json", summary)
    figures = [
        observation_figure(epoch_rows, snr_rows, output_directory / "observations.png"),
        observation_signal_coverage_figure(
            signal_rows, output_directory / "observation_signal_coverage.png"
        ),
        observation_snr_histogram_figure(
            snr_rows, output_directory / "observation_snr_histograms.png"
        ),
        observation_elevation_snr_figure(
            snr_elevation_rows, output_directory / "observation_snr_by_elevation.png"
        ),
        observation_tracking_figure(
            lock_arc_rows, cycle_slip_rows, output_directory / "observation_tracking.png"
        ),
        observation_reception_figure(
            reception_rows, ssr_telemetry_rows,
            output_directory / "observation_reception_integrity.png",
        ),
    ]
    figures = [figure for figure in figures if figure is not None]
    write_html_report(
        output_directory / "report.html", "Observation Analysis", summary, figures, warnings
    )
    write_manifest(output_directory, "observations", config_path, config, input_paths, warnings)
    return summary
