"""Parses RINEX observation epochs and signal values for analysis."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import re

from ..core.time import calendar_epoch_to_utc, datetime_to_unix_ns
from ..models import ObservationValue


class RinexParseError(ValueError):
    pass


_SATELLITE_PATTERN = re.compile(r"^[A-Z][0-9]{2}$")
_OBSERVATION_TYPE_PATTERN = re.compile(r"^[A-Z][0-9][A-Z0-9]$")


@dataclass(frozen=True)
class RinexObservationFile:
    observations: list[ObservationValue]
    epoch_satellites: dict[int, set[str]]
    observation_types: dict[str, list[str]]
    event_epochs: list[tuple[int, int, int]]


def _parse_header(lines: list[str], path: Path) -> tuple[dict[str, list[str]], int]:
    observation_types: dict[str, list[str]] = {}
    expected_counts: dict[str, int] = {}
    for index, line in enumerate(lines):
        label = line[60:].strip() if len(line) >= 60 else ""
        if label == "SYS / # / OBS TYPES":
            system = line[:1].strip().upper()
            if system:
                count_text = line[3:6].strip()
                if count_text:
                    try:
                        count = int(count_text)
                    except ValueError as exc:
                        raise RinexParseError(
                            f"Invalid observation type count at {path}:{index + 1}"
                        ) from exc
                    if count <= 0:
                        raise RinexParseError(
                            f"Invalid observation type count at {path}:{index + 1}"
                        )
                    if system in expected_counts and observation_types[system]:
                        raise RinexParseError(
                            f"Duplicate observation type header for {system} at {path}:{index + 1}"
                        )
                    expected_counts[system] = count
                    observation_types.setdefault(system, [])
                elif system not in expected_counts:
                    raise RinexParseError(
                        f"Missing observation type count at {path}:{index + 1}"
                    )
            else:
                incomplete = [
                    key
                    for key, count in expected_counts.items()
                    if len(observation_types[key]) < count
                ]
                if not incomplete:
                    raise RinexParseError(
                        f"Unexpected observation type continuation at {path}:{index + 1}"
                    )
                system = incomplete[-1]
            values = line[6:60].split()
            observation_types[system].extend(values)
            if len(observation_types[system]) > expected_counts[system]:
                raise RinexParseError(
                    f"Too many observation types for {system} at {path}:{index + 1}"
                )
        if label == "END OF HEADER" or "END OF HEADER" in line:
            for system, count in expected_counts.items():
                values = observation_types[system]
                if len(values) != count:
                    raise RinexParseError(
                        f"Observation type count mismatch for {system} in {path}"
                    )
                if len(set(values)) != len(values) or any(
                    not _OBSERVATION_TYPE_PATTERN.fullmatch(value) for value in values
                ):
                    raise RinexParseError(f"Invalid observation types for {system} in {path}")
            if not observation_types:
                raise RinexParseError(f"RINEX header declares no observation types: {path}")
            return observation_types, index + 1
    raise RinexParseError(f"RINEX header is incomplete: {path}")


def _observation_fields(line: str) -> list[str]:
    payload = line[3:]
    slot_count = max(4, math.ceil(len(payload) / 16))
    payload = payload.ljust(slot_count * 16)
    return [payload[index * 16 : (index + 1) * 16] for index in range(slot_count)]


def _quality_digit(value: str, path: Path, line_number: int, name: str) -> int | None:
    if not value.strip():
        return None
    if not value.isdigit():
        raise RinexParseError(f"Invalid {name} at {path}:{line_number}")
    return int(value)


def read_rinex_observation_file(
    path: Path, time_scale: str = "UTC"
) -> RinexObservationFile:
    if not path.is_file():
        raise FileNotFoundError(path)
    lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
    observation_types, index = _parse_header(lines, path)

    observations: list[ObservationValue] = []
    seen_observations: set[tuple[int, str, str]] = set()
    epoch_satellites: dict[int, set[str]] = {}
    event_epochs: list[tuple[int, int, int]] = []
    while index < len(lines):
        line = lines[index]
        line_number = index + 1
        if not line.strip():
            index += 1
            continue
        if not line.startswith(">"):
            raise RinexParseError(
                f"Observation record appears before an epoch at {path}:{line_number}"
            )
        parts = line[1:].split()
        if len(parts) < 8:
            raise RinexParseError(f"Invalid RINEX epoch at {path}:{line_number}")
        try:
            utc = calendar_epoch_to_utc(
                int(parts[0]),
                int(parts[1]),
                int(parts[2]),
                int(parts[3]),
                int(parts[4]),
                float(parts[5]),
                time_scale,
            )
            flag = int(parts[6])
            record_count = int(parts[7])
        except (ValueError, IndexError) as exc:
            raise RinexParseError(f"Invalid RINEX epoch at {path}:{line_number}") from exc
        if flag < 0 or flag > 6 or record_count < 0:
            raise RinexParseError(f"Invalid RINEX epoch at {path}:{line_number}")
        epoch_ns = datetime_to_unix_ns(utc)
        index += 1
        if flag not in {0, 1}:
            if index + record_count > len(lines):
                raise RinexParseError(f"Truncated event epoch at {path}:{line_number}")
            event_epochs.append((epoch_ns, flag, record_count))
            index += record_count
            continue

        satellites = epoch_satellites.setdefault(epoch_ns, set())
        for _ in range(record_count):
            if index >= len(lines) or lines[index].startswith(">"):
                raise RinexParseError(f"Truncated observation epoch at {path}:{line_number}")
            first_line_number = index + 1
            satellite = lines[index][:3].strip().upper()
            if not _SATELLITE_PATTERN.fullmatch(satellite):
                raise RinexParseError(
                    f"Invalid RINEX satellite at {path}:{first_line_number}"
                )
            types = observation_types.get(satellite[0])
            if types is None:
                raise RinexParseError(
                    f"No observation types declared for {satellite} at {path}:{first_line_number}"
                )
            fields = _observation_fields(lines[index])
            field_lines = [first_line_number] * len(fields)
            index += 1
            while len(fields) < len(types):
                if index >= len(lines) or lines[index].startswith(">"):
                    raise RinexParseError(
                        f"Truncated observation continuation at {path}:{first_line_number}"
                    )
                if lines[index][:3].strip():
                    raise RinexParseError(
                        f"Missing observation continuation at {path}:{index + 1}"
                    )
                continued = _observation_fields(lines[index])
                fields.extend(continued)
                field_lines.extend([index + 1] * len(continued))
                index += 1
            for observation_type, field, field_line in zip(types, fields, field_lines):
                value_text = field[:14].strip()
                lli = _quality_digit(field[14:15], path, field_line, "LLI")
                ssi = _quality_digit(field[15:16], path, field_line, "SSI")
                if not value_text:
                    continue
                try:
                    value = float(value_text.replace("D", "E").replace("d", "e"))
                except ValueError as exc:
                    raise RinexParseError(f"Invalid observation at {path}:{field_line}") from exc
                if not math.isfinite(value):
                    raise RinexParseError(f"Non-finite observation at {path}:{field_line}")
                key = (epoch_ns, satellite, observation_type)
                if key in seen_observations:
                    raise RinexParseError(
                        f"Duplicate observation {satellite}/{observation_type} at "
                        f"{path}:{field_line}"
                    )
                seen_observations.add(key)
                observations.append(
                    ObservationValue(
                        epoch_ns, satellite, observation_type, value, lli=lli, ssi=ssi
                    )
                )
            if satellite in satellites:
                raise RinexParseError(
                    f"Duplicate satellite record {satellite} at {path}:{first_line_number}"
                )
            satellites.add(satellite)

    return RinexObservationFile(
        observations=observations,
        epoch_satellites=epoch_satellites,
        observation_types=observation_types,
        event_epochs=event_epochs,
    )


def read_rinex_observations(path: Path, time_scale: str = "UTC") -> list[ObservationValue]:
    return read_rinex_observation_file(path, time_scale).observations
