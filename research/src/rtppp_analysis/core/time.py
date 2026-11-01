"""Provides UTC, GPS week, and day-boundary conversion utilities."""

from __future__ import annotations

from datetime import datetime, timedelta, timezone


UTC = timezone.utc
GPS_EPOCH = datetime(1980, 1, 6, tzinfo=UTC)
UNIX_EPOCH = datetime(1970, 1, 1, tzinfo=UTC)

# Effective UTC dates and GPS-UTC offsets after each leap second.
_GPS_UTC_OFFSETS = (
    (datetime(1981, 7, 1, tzinfo=UTC), 1),
    (datetime(1982, 7, 1, tzinfo=UTC), 2),
    (datetime(1983, 7, 1, tzinfo=UTC), 3),
    (datetime(1985, 7, 1, tzinfo=UTC), 4),
    (datetime(1988, 1, 1, tzinfo=UTC), 5),
    (datetime(1990, 1, 1, tzinfo=UTC), 6),
    (datetime(1991, 1, 1, tzinfo=UTC), 7),
    (datetime(1992, 7, 1, tzinfo=UTC), 8),
    (datetime(1993, 7, 1, tzinfo=UTC), 9),
    (datetime(1994, 7, 1, tzinfo=UTC), 10),
    (datetime(1996, 1, 1, tzinfo=UTC), 11),
    (datetime(1997, 7, 1, tzinfo=UTC), 12),
    (datetime(1999, 1, 1, tzinfo=UTC), 13),
    (datetime(2006, 1, 1, tzinfo=UTC), 14),
    (datetime(2009, 1, 1, tzinfo=UTC), 15),
    (datetime(2012, 7, 1, tzinfo=UTC), 16),
    (datetime(2015, 7, 1, tzinfo=UTC), 17),
    (datetime(2017, 1, 1, tzinfo=UTC), 18),
)


def gps_utc_offset(utc: datetime) -> int:
    if utc.tzinfo is None:
        raise ValueError("UTC datetime must include timezone information")
    value = utc.astimezone(UTC)
    offset = 0
    for effective, candidate in _GPS_UTC_OFFSETS:
        if value < effective:
            break
        offset = candidate
    return offset


def datetime_to_unix_ns(value: datetime) -> int:
    if value.tzinfo is None:
        raise ValueError("Datetime must include timezone information")
    utc = value.astimezone(UTC)
    delta = utc - UNIX_EPOCH
    return ((delta.days * 86400 + delta.seconds) * 1_000_000_000) + delta.microseconds * 1000


def unix_ns_to_datetime(value: int) -> datetime:
    seconds, nanoseconds = divmod(int(value), 1_000_000_000)
    return UNIX_EPOCH + timedelta(seconds=seconds, microseconds=nanoseconds // 1000)


def gps_week_sow_to_utc(week: int, sow: float) -> datetime:
    gps_label = GPS_EPOCH + timedelta(weeks=week, seconds=sow)
    utc = gps_label
    for _ in range(3):
        utc = gps_label - timedelta(seconds=gps_utc_offset(utc))
    return utc


def utc_to_gps_week_sow(value: datetime) -> tuple[int, float]:
    if value.tzinfo is None:
        raise ValueError("UTC datetime must include timezone information")
    utc = value.astimezone(UTC)
    gps = utc + timedelta(seconds=gps_utc_offset(utc))
    total = (gps - GPS_EPOCH).total_seconds()
    week = int(total // 604800.0)
    return week, total - week * 604800.0


def calendar_epoch_to_utc(
    year: int,
    month: int,
    day: int,
    hour: int,
    minute: int,
    second: float,
    time_scale: str,
) -> datetime:
    whole_second = int(second)
    microsecond = int(round((second - whole_second) * 1_000_000))
    if microsecond >= 1_000_000:
        whole_second += 1
        microsecond -= 1_000_000
    label = datetime(year, month, day, hour, minute, 0, tzinfo=UTC) + timedelta(
        seconds=whole_second, microseconds=microsecond
    )
    scale = time_scale.strip().upper()
    if scale == "UTC":
        return label
    if scale != "GPS":
        raise ValueError(f"Unsupported time scale: {time_scale}")
    utc = label
    for _ in range(3):
        utc = label - timedelta(seconds=gps_utc_offset(utc))
    return utc
