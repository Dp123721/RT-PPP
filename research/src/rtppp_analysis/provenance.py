"""Collects reproducibility metadata and hashes for analysis inputs."""

from __future__ import annotations

import hashlib
import platform
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable

from . import __version__
from .io.tables import write_json


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def write_manifest(
    output_directory: Path,
    analysis: str,
    config_path: Path,
    config: dict[str, Any],
    input_paths: Iterable[Path],
    warnings: list[str],
) -> None:
    inputs = []
    for path in sorted({item.resolve() for item in input_paths}, key=str):
        inputs.append(
            {
                "path": str(path),
                "exists": path.is_file(),
                "size_bytes": path.stat().st_size if path.is_file() else None,
                "sha256": sha256_file(path) if path.is_file() else None,
            }
        )
    manifest = {
        "analysis": analysis,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "rtppp_analysis_version": __version__,
        "python": sys.version,
        "platform": platform.platform(),
        "config_path": str(config_path),
        "config_sha256": sha256_file(config_path),
        "config": config,
        "inputs": inputs,
        "warnings": warnings,
    }
    write_json(output_directory / "manifest.json", manifest)
