"""Validates research inputs and outputs against schemas and data contracts."""

from __future__ import annotations

import json
import math
from datetime import datetime
from importlib import resources
from pathlib import Path
from typing import Any

from jsonschema import Draft202012Validator, FormatChecker

from .config import ConfigError, ContractError
from .core.time import utc_to_gps_week_sow


_CONFIG_SCHEMAS = {
    "orbit": "orbit-config.schema.json",
    "observations": "observations-config.schema.json",
    "ppp": "ppp-config.schema.json",
}
_METRICS_SCHEMAS = {
    "orbit_clock": "orbit-metrics.schema.json",
    "observations": "observations-metrics.schema.json",
    "ppp": "ppp-metrics.schema.json",
}


def _load_schema(name: str) -> dict[str, Any]:
    source_path = Path(__file__).resolve().parents[2] / "schemas" / name
    try:
        if source_path.is_file():
            text = source_path.read_text(encoding="utf-8")
        else:
            resource = resources.files("rtppp_analysis.schemas").joinpath(name)
            text = resource.read_text(encoding="utf-8")
        value = json.loads(text)
    except (OSError, json.JSONDecodeError) as exc:
        raise ContractError(f"Cannot load Schema {name}: {exc}") from exc
    except (ImportError, ModuleNotFoundError) as exc:
        raise ContractError(f"Schema resource is unavailable: {name}") from exc
    Draft202012Validator.check_schema(value)
    return value


def _json_path(error: Any) -> str:
    parts = [str(part) if isinstance(part, int) else str(part) for part in error.absolute_path]
    return "$" + "".join(f"[{part}]" if part.isdigit() else f".{part}" for part in parts)


def _reject_non_finite(value: Any, path: str = "$") -> None:
    if isinstance(value, float) and not math.isfinite(value):
        raise ContractError(f"{path}: non-finite numbers are not valid JSON data")
    if isinstance(value, dict):
        for key, item in value.items():
            _reject_non_finite(item, f"{path}.{key}")
    elif isinstance(value, (list, tuple)):
        for index, item in enumerate(value):
            _reject_non_finite(item, f"{path}[{index}]")


def _validate(schema_name: str, value: Any) -> None:
    _reject_non_finite(value)
    validator = Draft202012Validator(_load_schema(schema_name), format_checker=FormatChecker())
    errors = sorted(validator.iter_errors(value), key=_json_path)
    if errors:
        error = errors[0]
        raise ContractError(f"{_json_path(error)}: {error.message}")


def _require_unique_ids(items: list[dict[str, Any]], path: str) -> None:
    seen: set[str] = set()
    for index, item in enumerate(items):
        raw_identifier = str(item["id"])
        identifier = raw_identifier.strip()
        if identifier != raw_identifier:
            raise ContractError(
                f"{path}[{index}].id: leading or trailing whitespace is not allowed"
            )
        if identifier in seen:
            raise ContractError(f"{path}[{index}].id: duplicate id {identifier!r}")
        seen.add(identifier)


def validate_config(command: str, value: Any) -> None:
    schema_name = _CONFIG_SCHEMAS.get(command)
    if schema_name is None:
        raise ConfigError(f"Unsupported analysis command: {command}")
    try:
        _validate(schema_name, value)
        if command == "orbit":
            products = [value["reference"], *value["candidates"]]
            _require_unique_ids(products, "$.products")
        else:
            _require_unique_ids(value["datasets"], "$.datasets")
    except ContractError as exc:
        raise ConfigError(f"Invalid {command} configuration: {exc}") from exc


def validate_compact(value: Any) -> None:
    _validate("rtppp-compact-v1.schema.json", value)
    time_base = value["meta"]["timeBase"]
    date = time_base["date"]
    epoch_text = time_base["epoch"]
    if epoch_text != f"{date}T00:00:00.000Z":
        raise ContractError(
            "$.meta.timeBase.epoch: must be UTC midnight for $.meta.timeBase.date"
        )
    epoch = datetime.fromisoformat(epoch_text.replace("Z", "+00:00"))
    expected_week, expected_sow = utc_to_gps_week_sow(epoch)
    if time_base["gpsWeek"] != expected_week:
        raise ContractError("$.meta.timeBase.gpsWeek: inconsistent with UTC base epoch")
    if not math.isclose(time_base["gpsSow"], expected_sow, rel_tol=0.0, abs_tol=1e-6):
        raise ContractError("$.meta.timeBase.gpsSow: inconsistent with UTC base epoch")


def validate_metrics(analysis: str, value: Any) -> None:
    schema_name = _METRICS_SCHEMAS.get(analysis)
    if schema_name is None:
        raise ContractError(f"Unsupported metrics contract: {analysis}")
    _validate(schema_name, value)
