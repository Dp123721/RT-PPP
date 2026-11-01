"""Defines the command-line interface for orbit, observation, and PPP analysis."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Any, Callable

from .config import ConfigError, load_config, resolve_path
from .contracts import validate_config
from .observations.analyze import run_observation_analysis
from .orbit.compare import run_orbit_analysis
from .ppp.analyze import run_ppp_analysis


Runner = Callable[[dict[str, Any], Path, Path], dict[str, Any]]


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="rtppp-analysis")
    subparsers = parser.add_subparsers(dest="command", required=True)
    for command, help_text in (
        ("orbit", "Compare SSR-corrected orbit/clock products"),
        ("observations", "Analyze RINEX observations and SNR"),
        ("ppp", "Analyze compact PPP JSON results"),
    ):
        child = subparsers.add_parser(command, help=help_text)
        child.add_argument("--config", required=True, help="Path to JSON configuration")
        child.add_argument("--output", help="Override output directory")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    runners: dict[str, Runner] = {
        "orbit": run_orbit_analysis,
        "observations": run_observation_analysis,
        "ppp": run_ppp_analysis,
    }
    try:
        config, config_path = load_config(args.config)
        validate_config(args.command, config)
        if args.output:
            output_directory = Path(args.output).expanduser().resolve()
        else:
            output_value = config.get("output_directory")
            if not isinstance(output_value, str) or not output_value.strip():
                raise ConfigError("Configuration requires output_directory")
            output_directory = resolve_path(output_value, config_path)
        summary = runners[args.command](config, config_path, output_directory)
        print(f"{args.command} analysis completed: {output_directory}")
        print(f"summary keys: {', '.join(sorted(summary))}")
        return 0
    except (ConfigError, FileNotFoundError, ValueError, OSError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
