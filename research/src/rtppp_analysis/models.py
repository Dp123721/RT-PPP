"""Defines typed records shared by the scientific analysis pipelines."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any

import numpy as np


class MissingReason(str, Enum):
    NOT_TRANSMITTED = "not_transmitted"
    NOT_RECEIVED = "not_received"
    STREAM_INTERRUPTION = "stream_interruption"
    STALE = "stale"
    IOD_MISMATCH = "iod_mismatch"
    REJECTED = "rejected"
    INVALID = "invalid"
    UNKNOWN = "unknown"


@dataclass
class OrbitPoint:
    epoch_ns: int
    satellite: str
    position_m: np.ndarray
    clock_s: float = float("nan")
    velocity_mps: np.ndarray | None = None


@dataclass(frozen=True)
class ClockPoint:
    epoch_ns: int
    satellite: str
    clock_s: float


@dataclass(frozen=True)
class ObservationValue:
    epoch_ns: int
    satellite: str
    observation_type: str
    value: float
    lli: int | None = None
    ssi: int | None = None

    @property
    def system(self) -> str:
        return self.satellite[:1]

    @property
    def signal(self) -> str:
        return self.observation_type[1:]


@dataclass
class PppEpoch:
    epoch_ns: int
    algorithm: str
    ar_status: str
    ecef_m: np.ndarray
    stored_enu_m: np.ndarray
    covariance_ecef_m2: np.ndarray
    ztd_m: float
    wet_residual_m: float
    ztd_std_m: float
    used_count: int
    eligible_count: int
    fixed_count: int
    hdop: float
    satellite_sets: list[list[Any]] = field(default_factory=list)
    ar_data: Any = None
    flags: int = 0


@dataclass
class Product:
    product_id: str
    time_scale: str
    orbit_reference: str
    reference_frame: str
    orbits: dict[str, list[OrbitPoint]]
    clocks: dict[str, list[ClockPoint]]
    input_paths: list[str]
