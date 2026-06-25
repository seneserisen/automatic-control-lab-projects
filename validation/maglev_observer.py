"""Observer-based magnetic-levitation reference experiment."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray
from scipy.signal import place_poles

from validation.numerics import rk4_step, rms, validate_time_step

FloatArray = NDArray[np.float64]


@dataclass(frozen=True)
class MaglevDesign:
    a: FloatArray
    b: FloatArray
    c: FloatArray
    controller_gain: FloatArray
    observer_gain: FloatArray
    prefilter: float
    equilibrium_state: FloatArray
    equilibrium_voltage: float
    controller_poles: FloatArray
    observer_poles: FloatArray
    observability_rank: int


@dataclass(frozen=True)
class MaglevObserverResult:
    time: FloatArray
    reference: FloatArray
    full_state_position: FloatArray
    observer_position: FloatArray
    estimated_position: FloatArray
    true_velocity: FloatArray
    estimated_velocity: FloatArray
    true_current_deviation: FloatArray
    estimated_current_deviation: FloatArray
    measured_gap: FloatArray
    measured_current: FloatArray
    observer_voltage: FloatArray
    full_state_tracking_rmse: float
    observer_tracking_rmse: float
    position_estimation_rmse: float
    velocity_estimation_rmse: float
    current_estimation_rmse: float
    maximum_voltage: float
    design: MaglevDesign


@dataclass(frozen=True)
class ConvergencePoint:
    dt: float
    tracking_rmse: float
    final_position: float
    maximum_voltage: float


def maglev_design() -> MaglevDesign:
    """Build the local model and deterministic controller/observer design."""
    mass, gravity, gap, magnetic_coefficient = 0.068, 9.81, 0.014, 6.53e-5
    resistance, inductance = 11.0, 0.4125
    equilibrium_current = np.sqrt(
        2.0 * mass * gravity * gap**2 / magnetic_coefficient
    )
    equilibrium_voltage = resistance * equilibrium_current
    a = np.array(
        [
            [0.0, 1.0, 0.0],
            [
                2.0 * gravity / gap,
                0.0,
                -magnetic_coefficient * equilibrium_current / (mass * gap**2),
            ],
            [0.0, 0.0, -resistance / inductance],
        ],
        dtype=float,
    )
    b = np.array([[0.0], [0.0], [1.0 / inductance]], dtype=float)
    c = np.array([[1.0, 0.0, 0.0], [0.0, 0.0, 1.0]], dtype=float)
    controller_gain = np.array(
        [[-6316.60910998, -168.35876027, 26.125]], dtype=float
    )
    prefilter = -1009.79192166
    observer_gain = place_poles(
        a.T, c.T, np.array([-80.0, -90.0, -100.0])
    ).gain_matrix.T
    observability = np.vstack((c, c @ a, c @ a @ a))
    return MaglevDesign(
        a=a,
        b=b,
        c=c,
        controller_gain=controller_gain,
        observer_gain=observer_gain,
        prefilter=prefilter,
        equilibrium_state=np.array([gap, 0.0, equilibrium_current], dtype=float),
        equilibrium_voltage=float(equilibrium_voltage),
        controller_poles=np.linalg.eigvals(a - b @ controller_gain),
        observer_poles=np.linalg.eigvals(a - observer_gain @ c),
        observability_rank=int(np.linalg.matrix_rank(observability)),
    )


def magnetic_levitation_observer_simulation(
    dt: float = 1e-4,
    *,
    seed: int = 42,
    gap_noise_std: float = 5e-6,
    current_noise_std: float = 2e-3,
) -> MaglevObserverResult:
    """Simulate full-state and observer-based control on the nonlinear plant."""
    validate_time_step(dt)
    if gap_noise_std < 0 or current_noise_std < 0:
        raise ValueError("measurement noise standard deviations must be non-negative")

    design = maglev_design()
    mass, gravity, magnetic_coefficient = 0.068, 9.81, 6.53e-5
    resistance, inductance = 11.0, 0.4125
    gap = design.equilibrium_state[0]

    time = np.arange(0.0, 0.5 + 0.5 * dt, dt)
    reference = np.zeros_like(time)
    reference[time >= 0.05] = 0.0005

    rng = np.random.default_rng(seed)
    measurement_noise = np.vstack(
        (
            rng.normal(0.0, gap_noise_std, time.size),
            rng.normal(0.0, current_noise_std, time.size),
        )
    )

    observer_plant = np.zeros((3, time.size))
    observer_plant[:, 0] = design.equilibrium_state
    estimate = np.zeros((3, time.size))
    full_state_plant = np.zeros((3, time.size))
    full_state_plant[:, 0] = design.equilibrium_state
    measured = np.zeros((2, time.size))
    observer_voltage = np.zeros(time.size)

    def plant_rhs(state: FloatArray, voltage: float) -> FloatArray:
        actual_gap = max(float(state[0]), 0.004)
        actual_current = max(float(state[2]), 0.0)
        magnetic_force = (
            magnetic_coefficient * actual_current**2 / (2.0 * actual_gap**2)
        )
        return np.array(
            [
                state[1],
                gravity - magnetic_force / mass,
                (voltage - resistance * state[2]) / inductance,
            ]
        )

    for index in range(time.size - 1):
        true_deviation = observer_plant[:, index] - design.equilibrium_state
        measured[:, index] = design.c @ true_deviation + measurement_noise[:, index]
        voltage_deviation = float(
            (
                -design.controller_gain @ estimate[:, index]
                + design.prefilter * reference[index]
            ).item()
        )
        observer_voltage[index] = np.clip(
            design.equilibrium_voltage + voltage_deviation, 0.0, 30.0
        )
        applied_deviation = observer_voltage[index] - design.equilibrium_voltage

        observer_plant[:, index + 1] = rk4_step(
            lambda _time, state, voltage=float(observer_voltage[index]): plant_rhs(
                state, voltage
            ),
            time[index],
            observer_plant[:, index],
            dt,
        )
        estimate[:, index + 1] = rk4_step(
            lambda _time, state, voltage=float(applied_deviation), output=measured[
                :, index
            ]: (
                design.a @ state
                + design.b[:, 0] * voltage
                + design.observer_gain @ (output - design.c @ state)
            ),
            time[index],
            estimate[:, index],
            dt,
        )

        full_deviation = full_state_plant[:, index] - design.equilibrium_state
        full_voltage_deviation = float(
            (
                -design.controller_gain @ full_deviation
                + design.prefilter * reference[index]
            ).item()
        )
        full_voltage = np.clip(
            design.equilibrium_voltage + full_voltage_deviation, 0.0, 30.0
        )
        full_state_plant[:, index + 1] = rk4_step(
            lambda _time, state, voltage=float(full_voltage): plant_rhs(state, voltage),
            time[index],
            full_state_plant[:, index],
            dt,
        )

    observer_voltage[-1] = observer_voltage[-2]
    final_deviation = observer_plant[:, -1] - design.equilibrium_state
    measured[:, -1] = design.c @ final_deviation + measurement_noise[:, -1]

    true_deviation = observer_plant - design.equilibrium_state[:, None]
    full_deviation = full_state_plant - design.equilibrium_state[:, None]
    estimation_error = true_deviation - estimate

    return MaglevObserverResult(
        time=time,
        reference=reference,
        full_state_position=full_deviation[0],
        observer_position=true_deviation[0],
        estimated_position=estimate[0],
        true_velocity=true_deviation[1],
        estimated_velocity=estimate[1],
        true_current_deviation=true_deviation[2],
        estimated_current_deviation=estimate[2],
        measured_gap=measured[0],
        measured_current=measured[1],
        observer_voltage=observer_voltage,
        full_state_tracking_rmse=rms(full_deviation[0] - reference),
        observer_tracking_rmse=rms(true_deviation[0] - reference),
        position_estimation_rmse=rms(estimation_error[0]),
        velocity_estimation_rmse=rms(estimation_error[1]),
        current_estimation_rmse=rms(estimation_error[2]),
        maximum_voltage=float(np.max(observer_voltage)),
        design=design,
    )


def maglev_observer_convergence(
    step_sizes: tuple[float, ...] = (8e-4, 4e-4, 2e-4, 1e-4, 5e-5),
) -> tuple[ConvergencePoint, ...]:
    """Run a no-noise fixed-step convergence study."""
    points: list[ConvergencePoint] = []
    for dt in step_sizes:
        result = magnetic_levitation_observer_simulation(
            dt,
            gap_noise_std=0.0,
            current_noise_std=0.0,
        )
        points.append(
            ConvergencePoint(
                dt=dt,
                tracking_rmse=result.observer_tracking_rmse,
                final_position=float(result.observer_position[-1]),
                maximum_voltage=result.maximum_voltage,
            )
        )
    return tuple(points)
