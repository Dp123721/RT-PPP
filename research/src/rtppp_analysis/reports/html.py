"""Renders analysis results and provenance as standalone HTML reports."""

from __future__ import annotations

import html
import math
from pathlib import Path
from typing import Any

from ..io.tables import json_value


def _label(key: str) -> str:
    labels = {
        "candidate_id": "Candidate",
        "reference_id": "Reference",
        "candidate_sample_count": "Candidate samples",
        "compared_orbit_count": "Orbit comparisons",
        "clock_compared_count": "Clock comparisons",
        "orbit_coverage_rate": "Orbit coverage",
        "clock_coverage_rate": "Clock coverage",
        "orbit_3d_m_rms": "3D orbit RMS (m)",
        "clock_datum_removed_m_rms": "Datum-removed clock RMS (m)",
        "sisre_m_rms": "SISRE RMS (m)",
        "sisre_m_coverage_rate": "SISRE coverage",
    }
    if key in labels:
        return labels[key]
    return key.replace("_", " ").title()


def _display(value: Any) -> str:
    normalized = json_value(value)
    if normalized is None:
        return "-"
    if isinstance(normalized, float):
        return f"{normalized:.6g}" if math.isfinite(normalized) else "-"
    if isinstance(normalized, (str, int, bool)):
        return str(normalized)
    if isinstance(normalized, list) and all(
        isinstance(item, (str, int, float, bool)) or item is None for item in normalized
    ):
        return ", ".join(_display(item) for item in normalized)
    return "See structured outputs"


def _overview_rows(summary: dict[str, Any]) -> list[tuple[str, Any]]:
    keys = ("reference", "candidates", "row_count", "common_sample_count", "common_epoch_count")
    rows = [(key, summary[key]) for key in keys if key in summary]
    definition = summary.get("convergence_definition")
    if isinstance(definition, dict):
        rows.extend((f"convergence {key}", value) for key, value in definition.items())
    return rows


def _table(title: str, rows: list[dict[str, Any]], columns: list[str], limit: int = 20) -> str:
    selected = rows[:limit]
    usable = [column for column in columns if any(column in row for row in selected)]
    if not usable:
        return ""
    header = "".join(f"<th>{html.escape(_label(column))}</th>" for column in usable)
    body = "".join(
        "<tr>"
        + "".join(f"<td>{html.escape(_display(row.get(column)))}</td>" for column in usable)
        + "</tr>"
        for row in selected
    )
    note = f"<p class=\"note\">Showing first {limit} rows.</p>" if len(rows) > limit else ""
    return (
        f"<section><h2>{html.escape(title)}</h2>{note}<div class=\"table-wrap\">"
        f"<table><thead><tr>{header}</tr></thead><tbody>{body}</tbody></table></div></section>"
    )


def _report_tables(summary: dict[str, Any]) -> str:
    sections: list[str] = []
    metrics = summary.get("metrics")
    if isinstance(metrics, list):
        sections.append(_table(
            "System Summary", [row for row in metrics if isinstance(row, dict)],
            ["scope", "candidate_id", "reference_id", "system", "sample_count", "satellite_count",
             "orbit_3d_m_rms", "clock_datum_removed_m_rms", "sisre_m_rms", "sisre_m_coverage_rate"],
        ))
    coverage = summary.get("coverage")
    if isinstance(coverage, list):
        sections.append(_table(
            "Comparison Coverage", [row for row in coverage if isinstance(row, dict)],
            ["comparison_scope", "candidate_id", "reference_id", "candidate_sample_count",
             "compared_orbit_count", "clock_compared_count", "orbit_coverage_rate",
             "clock_coverage_rate", "rejection_counts"],
        ))
    datasets = summary.get("datasets")
    if isinstance(datasets, list):
        sections.append(_table(
            "Dataset Summary", [row for row in datasets if isinstance(row, dict)],
            ["dataset_id", "algorithm_label", "station", "epoch_count", "solution_count",
             "finite_solution_count", "fix_rate", "convergence_seconds"],
        ))
    paired = summary.get("paired_comparisons")
    if isinstance(paired, list):
        sections.append(_table(
            "Paired Comparisons", [row for row in paired if isinstance(row, dict)],
            ["candidate_id", "reference_id", "common_epoch_count", "horizontal_difference_rms_m",
             "vertical_difference_rms_m", "fix_rate_difference"],
        ))
    signals = summary.get("signal_groups")
    if isinstance(signals, list):
        sections.append(_table(
            "Signal Coverage", [row for row in signals if isinstance(row, dict)],
            ["dataset_id", "system", "observation_type", "value_coverage_rate",
             "epoch_coverage_rate", "satellite_coverage_rate"],
        ))
    return "".join(section for section in sections if section)


def write_html_report(
    path: Path,
    title: str,
    summary: dict[str, Any],
    figures: list[Path],
    warnings: list[str],
) -> None:
    figure_html = "\n".join(
        f'<figure><img src="{html.escape(figure.name)}" alt="{html.escape(figure.stem)}">'
        f"<figcaption>{html.escape(figure.stem)}</figcaption></figure>"
        for figure in figures
        if figure.is_file()
    )
    warning_html = "".join(f"<li>{html.escape(item)}</li>" for item in warnings)
    overview_html = "".join(
        f"<dt>{html.escape(_label(key))}</dt><dd>{html.escape(_display(value))}</dd>"
        for key, value in _overview_rows(summary)
    )
    document = f"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>{html.escape(title)}</title>
<style>
body{{font:14px/1.5 Arial,sans-serif;margin:24px;color:#1f2937;background:#fff}}
main{{max-width:1280px;margin:auto}}h1,h2{{color:#111827}}section{{margin:28px 0}}
dl{{display:grid;grid-template-columns:max-content 1fr;gap:4px 20px;margin:0}}dt{{font-weight:700}}dd{{margin:0}}
.table-wrap{{overflow-x:auto}}table{{border-collapse:collapse;width:100%;font-size:13px}}th,td{{padding:7px 9px;border-bottom:1px solid #d1d5db;text-align:left;white-space:nowrap}}th{{background:#f3f4f6}}.note{{color:#4b5563}}
figure{{margin:24px 0}}img{{max-width:100%;height:auto;border:1px solid #d1d5db}}li{{margin:4px 0}}
</style>
</head><body><main><h1>{html.escape(title)}</h1>
<section><h2>Run Overview</h2><dl>{overview_html or '<dt>Result</dt><dd>See structured outputs.</dd>'}</dl></section>
<section><h2>Validation Warnings</h2><ul>{warning_html or '<li>None</li>'}</ul></section>
{_report_tables(summary)}
<section><h2>Figures</h2>{figure_html or '<p>No figures generated.</p>'}</section>
<p class="note">Full metrics and all per-satellite or per-signal fields are available in metrics.json and the CSV outputs.</p>
</main></body></html>"""
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(document, encoding="utf-8")
