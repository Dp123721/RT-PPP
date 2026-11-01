"""Shared synthetic fixtures and helpers for research-package tests."""

from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import Any


RESEARCH_ROOT = Path(__file__).resolve().parents[1]
SOURCE_ROOT = RESEARCH_ROOT / "src"
if str(SOURCE_ROOT) not in sys.path:
    sys.path.insert(0, str(SOURCE_ROOT))


def compact_solution(offset_ms: int = 0) -> list[Any]:
    return [
        offset_ms,
        2,
        1,
        [6378137.0, 0.0, 0.0],
        [0.0, 0.0, 0.0],
        [0.01, 0.0, 0.0, 0.01, 0.0, 0.04],
        [2.4, 0.2, 0.01],
        [8, 4, 0, 1.2],
        [list(range(1, 9)), list(range(1, 5)), []],
        [100, 2.5, 0, []],
        0,
    ]


def compact_root(solutions: list[list[Any]] | None = None) -> dict[str, Any]:
    return {
        "meta": {
            "schema": "rtppp.compact",
            "version": 1,
            "station": {
                "id": "TEST",
                "fileId": "TEST",
                "referenceEcef": [6378137.0, 0.0, 0.0],
            },
            "timeBase": {
                "system": "UTC",
                "date": "2026-07-27",
                "epoch": "2026-07-27T00:00:00.000Z",
                "unit": "ms",
                "gpsWeek": 2429,
                "gpsSow": 86418.0,
            },
            "algorithms": ["NONE", "LAMBDA", "BIE"],
            "arStatus": ["disabled", "float", "partial_fix", "fixed"],
            "systems": ["?", "G", "R", "E", "C", "J", "S", "I"],
            "ambiguityGroups": ["", "L1", "L2", "NL", "WL"],
            "satelliteIdEncoding": {
                "formula": "systemBase + PRN",
                "systemBase": {
                    "G": 0,
                    "R": 100,
                    "E": 200,
                    "C": 300,
                    "J": 400,
                    "S": 500,
                    "I": 600,
                },
                "unknown": "original string",
            },
            "layout": {
                "solution": [
                    "timeOffsetMs",
                    "algorithm",
                    "arStatus",
                    "ecefM",
                    "enuM",
                    "ecefCovarianceM2",
                    "troposphere",
                    "satelliteSummary",
                    "satelliteSets",
                    "arData",
                    "flags",
                ],
                "ecefCovarianceM2": ["qxx", "qxy", "qxz", "qyy", "qyz", "qzz"],
                "troposphere": ["ztdM", "wetResidualM", "ztdStdM"],
                "satelliteSummary": ["used", "arEligible", "fixed", "hdop"],
                "satelliteSets": ["used", "arEligible", "fixed"],
                "lambdaArData": ["ratio", "bootstrap", "constraintsApplied", "groups"],
                "bieArData": [
                    "candidateCount",
                    "effectiveCandidates",
                    "constraintsApplied",
                    "groups",
                ],
                "arGroup": [
                    "system",
                    "ambiguityGroup",
                    "referenceSatellite",
                    "fixed",
                    "eligible",
                    "maxFractionCycle",
                    "maxSigmaCycle",
                ],
            },
        },
        "solutions": solutions if solutions is not None else [compact_solution()],
    }


def write_json(path: Path, value: Any) -> None:
    path.write_text(json.dumps(value, allow_nan=False), encoding="utf-8")


def rinex_header_line(content: str, label: str) -> str:
    return content.ljust(60) + label


def rinex_observation(
    value: float | None, lli: int | None = None, ssi: int | None = None
) -> str:
    value_text = " " * 14 if value is None else f"{value:14.3f}"
    lli_text = " " if lli is None else str(lli)
    ssi_text = " " if ssi is None else str(ssi)
    return value_text + lli_text + ssi_text
