"""Provides ECEF/geodetic conversions and local NEU coordinate transforms."""

from __future__ import annotations

import math

import numpy as np


WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)


def ecef_to_geodetic(ecef_m: np.ndarray) -> tuple[float, float, float]:
    x, y, z = np.asarray(ecef_m, dtype=float)
    longitude = math.atan2(y, x)
    horizontal = math.hypot(x, y)
    latitude = math.atan2(z, horizontal * (1.0 - WGS84_E2))
    height = 0.0
    for _ in range(12):
        sin_lat = math.sin(latitude)
        prime_vertical = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
        cos_lat = math.cos(latitude)
        if abs(cos_lat) < 1e-15:
            height = abs(z) - prime_vertical * (1.0 - WGS84_E2)
        else:
            height = horizontal / cos_lat - prime_vertical
        denominator = horizontal * (1.0 - WGS84_E2 * prime_vertical / (prime_vertical + height))
        updated = math.atan2(z, denominator)
        if abs(updated - latitude) < 1e-14:
            latitude = updated
            break
        latitude = updated
    return latitude, longitude, height


def ecef_delta_to_enu(delta_ecef_m: np.ndarray, reference_ecef_m: np.ndarray) -> np.ndarray:
    latitude, longitude, _ = ecef_to_geodetic(reference_ecef_m)
    sin_lat, cos_lat = math.sin(latitude), math.cos(latitude)
    sin_lon, cos_lon = math.sin(longitude), math.cos(longitude)
    rotation = np.array(
        [
            [-sin_lon, cos_lon, 0.0],
            [-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat],
            [cos_lat * cos_lon, cos_lat * sin_lon, sin_lat],
        ],
        dtype=float,
    )
    return rotation @ np.asarray(delta_ecef_m, dtype=float)


def covariance_ecef_to_enu(covariance: np.ndarray, reference_ecef_m: np.ndarray) -> np.ndarray:
    latitude, longitude, _ = ecef_to_geodetic(reference_ecef_m)
    sin_lat, cos_lat = math.sin(latitude), math.cos(latitude)
    sin_lon, cos_lon = math.sin(longitude), math.cos(longitude)
    e_n_u = np.array(
        [
            [-sin_lon, cos_lon, 0.0],
            [-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat],
            [cos_lat * cos_lon, cos_lat * sin_lon, sin_lat],
        ]
    )
    matrix = np.asarray(covariance, dtype=float)
    if matrix.shape != (3, 3):
        raise ValueError("ECEF covariance must be a 3-by-3 matrix")
    return e_n_u @ matrix @ e_n_u.T


def rac_basis(position_m: np.ndarray, velocity_mps: np.ndarray) -> np.ndarray:
    position = np.asarray(position_m, dtype=float)
    velocity = np.asarray(velocity_mps, dtype=float)
    radial = position / np.linalg.norm(position)
    cross = np.cross(position, velocity)
    cross_norm = np.linalg.norm(cross)
    if not np.isfinite(cross_norm) or cross_norm < 1e-12:
        raise ValueError("Cannot form RAC basis from collinear position and velocity")
    cross /= cross_norm
    along = np.cross(cross, radial)
    along /= np.linalg.norm(along)
    return np.vstack((radial, along, cross))


def ecef_delta_to_rac(
    delta_ecef_m: np.ndarray, reference_position_m: np.ndarray, reference_velocity_mps: np.ndarray
) -> np.ndarray:
    return rac_basis(reference_position_m, reference_velocity_mps) @ np.asarray(
        delta_ecef_m, dtype=float
    )
