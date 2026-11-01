"""Loads, validates, and resolves research pipeline configuration."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any


class ConfigError(ValueError):
    pass


class ContractError(ValueError):
    pass


def load_config(path: str | Path) -> tuple[dict[str, Any], Path]:
    config_path = Path(path).expanduser().resolve()
    if not config_path.is_file():
        raise ConfigError(f"Configuration file does not exist: {config_path}")
    try:
        data = json.loads(config_path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise ConfigError(f"Invalid JSON configuration {config_path}: {exc}") from exc
    if not isinstance(data, dict):
        raise ConfigError("Top-level configuration must be an object")
    return data, config_path


def resolve_path(value: str, config_path: Path) -> Path:
    path = Path(value).expanduser()
    if not path.is_absolute():
        path = config_path.parent / path
    return path.resolve()


def require(config: dict[str, Any], key: str, expected_type: type | tuple[type, ...]) -> Any:
    if key not in config:
        raise ConfigError(f"Missing required configuration key: {key}")
    value = config[key]
    if not isinstance(value, expected_type):
        raise ConfigError(f"Configuration key {key!r} has an invalid type")
    return value
