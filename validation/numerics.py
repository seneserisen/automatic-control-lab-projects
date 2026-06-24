"""Shared numerical utilities for deterministic control simulations."""

from __future__ import annotations

from collections.abc import Callable

import numpy as np
from numpy.typing import NDArray

FloatArray = NDArray[np.float64]


def validate_time_step(dt: float) -> None:
    """Reject non-positive or non-finite integration steps."""
    if not np.isfinite(dt) or dt <= 0.0:
        raise ValueError("dt must be a positive finite number")


def rk4_step(
    rhs: Callable[[float, FloatArray], FloatArray],
    time: float,
    state: FloatArray,
    dt: float,
) -> FloatArray:
    """Advance one classical fourth-order Runge-Kutta step."""
    validate_time_step(dt)
    k1 = rhs(time, state)
    k2 = rhs(time + 0.5 * dt, state + 0.5 * dt * k1)
    k3 = rhs(time + 0.5 * dt, state + 0.5 * dt * k2)
    k4 = rhs(time + dt, state + dt * k3)
    return state + dt * (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0


def rms(values: FloatArray) -> float:
    """Return the root-mean-square value."""
    array = np.asarray(values, dtype=float)
    if array.size == 0:
        raise ValueError("values must not be empty")
    return float(np.sqrt(np.mean(np.square(array))))


def peak_abs(values: FloatArray) -> float:
    """Return the largest absolute magnitude."""
    array = np.asarray(values, dtype=float)
    if array.size == 0:
        raise ValueError("values must not be empty")
    return float(np.max(np.abs(array)))


def sustained_entry_time(
    time: FloatArray,
    error: FloatArray,
    *,
    tolerance: float,
    start_time: float,
    hold_time: float,
    end_time: float | None = None,
) -> float:
    """Return the first time an error remains within tolerance for hold_time.

    The result is ``nan`` when no sustained interval is found.
    """
    if tolerance <= 0.0 or hold_time <= 0.0:
        raise ValueError("tolerance and hold_time must be positive")
    time_array = np.asarray(time, dtype=float)
    error_array = np.asarray(error, dtype=float)
    if time_array.ndim != 1 or error_array.shape != time_array.shape:
        raise ValueError("time and error must be one-dimensional arrays of equal length")
    if time_array.size < 2:
        raise ValueError("at least two time samples are required")

    dt = float(np.median(np.diff(time_array)))
    window = max(1, int(np.ceil(hold_time / dt)))
    valid = (time_array >= start_time) & (np.abs(error_array) <= tolerance)
    if end_time is not None:
        valid &= time_array <= end_time

    run_length = 0
    for index, inside in enumerate(valid):
        run_length = run_length + 1 if inside else 0
        if run_length >= window:
            return float(time_array[index - window + 1])
    return float("nan")
