"""Numerical reference models for the control-laboratory portfolio.

The MATLAB scripts are the main project implementations. These functions mirror
key equations so GitHub Actions can verify the repository without a MATLAB
licence.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import NamedTuple

import numpy as np
from scipy.linalg import solve_continuous_are


class NonlinearResult(NamedTuple):
    stable_eigenvalues: np.ndarray
    unstable_eigenvalues: np.ndarray
    closed_loop_eigenvalues: np.ndarray


def nonlinear_linearisation() -> NonlinearResult:
    """Return Jacobian eigenvalues at the two selected equilibria."""

    def jacobian(x: np.ndarray) -> np.ndarray:
        return np.array(
            [[-3.0 * x[0] ** 2, 1.0], [2.0 * x[0] - 1.0, 2.0 * x[1] - 1.0]],
            dtype=float,
        )

    a_stable = jacobian(np.array([0.0, 0.0]))
    a_unstable = jacobian(np.array([1.0, 1.0]))
    b = np.array([[0.0], [1.0]])
    k = np.array([[1.0, 3.0]])
    return NonlinearResult(
        np.linalg.eigvals(a_stable),
        np.linalg.eigvals(a_unstable),
        np.linalg.eigvals(a_unstable - b @ k),
    )


class RotaryResult(NamedTuple):
    time: np.ndarray
    reference: np.ndarray
    response: np.ndarray
    feedforward: np.ndarray
    control: np.ndarray


def rotary_arm_simulation(dt: float = 0.002) -> RotaryResult:
    a1, a2, b = 6.0, 2.0, 1.0
    transition_time, final_time = 3.0, 5.0
    time = np.arange(0.0, final_time + dt, dt)
    tau = np.clip(time / transition_time, 0.0, 1.0)

    reference = 10 * tau**3 - 15 * tau**4 + 6 * tau**5
    r_dot = (30 * tau**2 - 60 * tau**3 + 30 * tau**4) / transition_time
    r_ddot = (60 * tau - 180 * tau**2 + 120 * tau**3) / transition_time**2
    r_dddot = (60 - 360 * tau + 360 * tau**2) / transition_time**3

    finished = time > transition_time
    r_dot[finished] = 0.0
    r_ddot[finished] = 0.0
    r_dddot[finished] = 0.0

    feedforward = (r_dddot + a2 * r_ddot + a1 * r_dot) / b
    gains = np.array([120.0, 68.0, 13.0])
    state = np.zeros((3, time.size))
    control = np.zeros(time.size)

    for idx in range(time.size - 1):
        target = np.array([reference[idx], r_dot[idx], r_ddot[idx]])
        control[idx] = np.clip(feedforward[idx] + gains @ (target - state[:, idx]), -25.0, 25.0)
        derivative = np.array(
            [
                state[1, idx],
                state[2, idx],
                -a1 * state[1, idx] - a2 * state[2, idx] + b * control[idx],
            ]
        )
        state[:, idx + 1] = state[:, idx] + dt * derivative
    control[-1] = control[-2]
    return RotaryResult(time, reference, state[0], feedforward, control)


@dataclass(frozen=True)
class SuspensionResult:
    time: np.ndarray
    road: np.ndarray
    passive_body: np.ndarray
    active_body: np.ndarray
    passive_acceleration: np.ndarray
    active_acceleration: np.ndarray
    active_force: np.ndarray
    gain: np.ndarray


def active_suspension_simulation(dt: float = 0.0005) -> SuspensionResult:
    ms, mu = 2.45, 1.40
    ks, kt = 900.0, 2500.0
    cs, ct = 12.0, 10.0

    a = np.array(
        [
            [0.0, 1.0, 0.0, 0.0],
            [-ks / ms, -cs / ms, ks / ms, cs / ms],
            [0.0, 0.0, 0.0, 1.0],
            [ks / mu, cs / mu, -(ks + kt) / mu, -(cs + ct) / mu],
        ]
    )
    b_u = np.array([[0.0], [1.0 / ms], [0.0], [-1.0 / mu]])
    b_r = np.array([[0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [kt / mu, ct / mu]])

    q = np.diag([1000.0, 50.0, 500.0, 20.0])
    r = np.array([[0.1]])
    p = solve_continuous_are(a, b_u, q, r)
    gain = np.linalg.solve(r, b_u.T @ p)

    time = np.arange(0.0, 3.0 + dt, dt)
    road = np.zeros_like(time)
    road_velocity = np.zeros_like(time)
    mask = (time >= 0.5) & (time <= 0.7)
    phase = (time[mask] - 0.5) / 0.2 * np.pi
    road[mask] = 0.02 * np.sin(phase)
    road_velocity[mask] = 0.02 * np.pi / 0.2 * np.cos(phase)

    passive = np.zeros((4, time.size))
    active = np.zeros((4, time.size))
    passive_acceleration = np.zeros(time.size)
    active_acceleration = np.zeros(time.size)
    active_force = np.zeros(time.size)

    for idx in range(time.size - 1):
        disturbance = np.array([road[idx], road_velocity[idx]])
        passive_derivative = a @ passive[:, idx] + b_r @ disturbance
        passive[:, idx + 1] = passive[:, idx] + dt * passive_derivative

        force = float(np.clip(-(gain @ active[:, idx]).item(), -50.0, 50.0))
        active_derivative = a @ active[:, idx] + b_u[:, 0] * force + b_r @ disturbance
        active[:, idx + 1] = active[:, idx] + dt * active_derivative

        passive_acceleration[idx] = passive_derivative[1]
        active_acceleration[idx] = active_derivative[1]
        active_force[idx] = force

    passive_acceleration[-1] = passive_acceleration[-2]
    active_acceleration[-1] = active_acceleration[-2]
    active_force[-1] = active_force[-2]

    return SuspensionResult(
        time,
        road,
        passive[0],
        active[0],
        passive_acceleration,
        active_acceleration,
        active_force,
        gain.ravel(),
    )


@dataclass(frozen=True)
class MaglevResult:
    time: np.ndarray
    reference: np.ndarray
    position: np.ndarray
    current: np.ndarray
    voltage: np.ndarray
    equilibrium_current: float
    open_loop_poles: np.ndarray
    closed_loop_poles: np.ndarray


def magnetic_levitation_simulation(dt: float = 0.0001) -> MaglevResult:
    mass, gravity, gap, magnetic_coefficient = 0.068, 9.81, 0.014, 6.53e-5
    resistance, inductance = 11.0, 0.4125
    equilibrium_current = np.sqrt(2 * mass * gravity * gap**2 / magnetic_coefficient)

    a_value = 2 * gravity / gap
    b_value = -9.804
    a = np.array([[0.0, 1.0, 0.0], [a_value, 0.0, b_value], [0.0, 0.0, -resistance / inductance]])
    b = np.array([[0.0], [0.0], [1.0 / inductance]])
    gain = np.array([[-6316.60910998, -168.35876027, 26.125]])
    nbar = -1009.79192166

    time = np.arange(0.0, 0.5 + dt, dt)
    reference = np.zeros_like(time)
    reference[time >= 0.05] = 0.001
    state = np.zeros((3, time.size))
    voltage = np.zeros(time.size)

    for idx in range(time.size - 1):
        voltage[idx] = np.clip(float((-gain @ state[:, idx]).item() + nbar * reference[idx]), -10.0, 10.0)
        derivative = a @ state[:, idx] + b[:, 0] * voltage[idx]
        state[:, idx + 1] = state[:, idx] + dt * derivative
    voltage[-1] = voltage[-2]

    return MaglevResult(
        time,
        reference,
        state[0],
        state[2],
        voltage,
        float(equilibrium_current),
        np.linalg.eigvals(a),
        np.linalg.eigvals(a - b @ gain),
    )


@dataclass(frozen=True)
class TankCase:
    h1: np.ndarray
    h2: np.ndarray
    saturated_control: np.ndarray
    integral_state: np.ndarray


@dataclass(frozen=True)
class TwoTankResult:
    time: np.ndarray
    reference: np.ndarray
    disturbance: np.ndarray
    no_anti_windup: TankCase
    anti_windup: TankCase


def two_tank_simulation(dt: float = 0.05) -> TwoTankResult:
    area_1 = area_2 = 50.0
    outlet_12, outlet_2, gravity = 0.20, 0.18, 981.0
    kp, ki, kaw = 5.0, 0.05, 0.30
    u_min, u_max = 0.0, 45.0

    time = np.arange(0.0, 700.0 + dt, dt)
    reference = np.full_like(time, 10.0)
    reference[(time >= 100.0) & (time < 300.0)] = 25.0
    disturbance = np.zeros_like(time)
    disturbance[(time >= 450.0) & (time < 520.0)] = -5.0

    def simulate(use_anti_windup: bool) -> TankCase:
        h1 = np.zeros_like(time)
        h2 = np.zeros_like(time)
        control = np.zeros_like(time)
        integral = np.zeros_like(time)

        for idx in range(time.size - 1):
            error = reference[idx] - h2[idx]
            unsaturated = kp * error + ki * integral[idx]
            control[idx] = np.clip(unsaturated, u_min, u_max)
            correction = kaw * (control[idx] - unsaturated) if use_anti_windup else 0.0
            integral[idx + 1] = integral[idx] + dt * (error + correction)

            flow_12 = outlet_12 * np.sqrt(2 * gravity * max(h1[idx] - h2[idx], 0.0))
            flow_out = outlet_2 * np.sqrt(2 * gravity * max(h2[idx], 0.0))
            dh1 = (control[idx] + disturbance[idx] - flow_12) / area_1
            dh2 = (flow_12 - flow_out) / area_2
            h1[idx + 1] = max(h1[idx] + dt * dh1, 0.0)
            h2[idx + 1] = max(h2[idx] + dt * dh2, 0.0)
        control[-1] = control[-2]
        return TankCase(h1, h2, control, integral)

    return TwoTankResult(time, reference, disturbance, simulate(False), simulate(True))


def repository_root() -> Path:
    return Path(__file__).resolve().parents[1]
