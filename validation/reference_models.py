"""Numerical reference models for the control-laboratory portfolio.

The MATLAB scripts are the main project implementations. These functions mirror
key equations so GitHub Actions can verify stability, physical constraints, and
published performance metrics without requiring a MATLAB licence.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
from numpy.typing import NDArray
from scipy.integrate import solve_ivp
from scipy.linalg import solve_continuous_are

from validation.numerics import peak_abs, rk4_step, rms, sustained_entry_time, validate_time_step

FloatArray = NDArray[np.float64]


@dataclass(frozen=True)
class NonlinearResult:
    stable_eigenvalues: FloatArray
    unstable_eigenvalues: FloatArray
    closed_loop_eigenvalues: FloatArray
    controllability_rank: int
    near_model_rmse: float
    far_model_rmse: float


def nonlinear_linearisation() -> NonlinearResult:
    """Analyse two equilibria and quantify local linear-model accuracy."""

    def jacobian(state: FloatArray) -> FloatArray:
        return np.array(
            [
                [-3.0 * state[0] ** 2, 1.0],
                [2.0 * state[0] - 1.0, 2.0 * state[1] - 1.0],
            ],
            dtype=float,
        )

    def nonlinear_rhs(_time: float, state: FloatArray, control: float) -> FloatArray:
        return np.array(
            [
                -(state[0] ** 3) + state[1],
                state[0] + state[1] - 2.0 + (1.0 - state[0]) ** 2 + (1.0 - state[1]) ** 2 + control,
            ]
        )

    stable_equilibrium = np.array([0.0, 0.0])
    unstable_equilibrium = np.array([1.0, 1.0])
    a_stable = jacobian(stable_equilibrium)
    a_unstable = jacobian(unstable_equilibrium)
    b = np.array([[0.0], [1.0]])
    gain = np.array([[1.0, 3.0]])
    closed_loop = a_unstable - b @ gain
    controllability = np.column_stack((b, a_unstable @ b))

    evaluation_time = np.linspace(0.0, 4.0, 801)

    def model_error(offset: FloatArray) -> float:
        initial = unstable_equilibrium + offset
        nonlinear = solve_ivp(
            lambda time, state: nonlinear_rhs(
                time,
                state,
                float((-gain @ (state - unstable_equilibrium)).item()),
            ),
            (evaluation_time[0], evaluation_time[-1]),
            initial,
            t_eval=evaluation_time,
            rtol=1e-9,
            atol=1e-11,
        ).y.T
        linear = (
            solve_ivp(
                lambda _time, deviation: closed_loop @ deviation,
                (evaluation_time[0], evaluation_time[-1]),
                offset,
                t_eval=evaluation_time,
                rtol=1e-10,
                atol=1e-12,
            ).y.T
            + unstable_equilibrium
        )
        return rms(nonlinear[:, 0] - linear[:, 0])

    return NonlinearResult(
        stable_eigenvalues=np.linalg.eigvals(a_stable),
        unstable_eigenvalues=np.linalg.eigvals(a_unstable),
        closed_loop_eigenvalues=np.linalg.eigvals(closed_loop),
        controllability_rank=int(np.linalg.matrix_rank(controllability)),
        near_model_rmse=model_error(np.array([0.02, -0.02])),
        far_model_rmse=model_error(np.array([0.25, -0.20])),
    )


@dataclass(frozen=True)
class RotaryResult:
    time: FloatArray
    reference: FloatArray
    feedback_only_response: FloatArray
    two_dof_response: FloatArray
    feedback_only_control: FloatArray
    two_dof_control: FloatArray
    disturbance: FloatArray
    feedback_only_rmse: float
    two_dof_rmse: float
    maximum_two_dof_control: float


def rotary_arm_simulation(dt: float = 0.002) -> RotaryResult:
    """Compare feedback-only and two-degree-of-freedom arm control."""
    validate_time_step(dt)
    a1, a2, input_gain = 6.0, 2.0, 1.0
    transition_time, final_time = 3.0, 6.0
    time = np.arange(0.0, final_time + dt, dt)
    tau = np.clip(time / transition_time, 0.0, 1.0)

    reference = 10.0 * tau**3 - 15.0 * tau**4 + 6.0 * tau**5
    reference_velocity = (30.0 * tau**2 - 60.0 * tau**3 + 30.0 * tau**4) / transition_time
    reference_acceleration = (60.0 * tau - 180.0 * tau**2 + 120.0 * tau**3) / transition_time**2
    reference_jerk = (60.0 - 360.0 * tau + 360.0 * tau**2) / transition_time**3
    finished = time > transition_time
    reference_velocity[finished] = 0.0
    reference_acceleration[finished] = 0.0
    reference_jerk[finished] = 0.0

    feedforward = (
        reference_jerk + a2 * reference_acceleration + a1 * reference_velocity
    ) / input_gain
    gains = np.array([120.0, 68.0, 13.0])
    disturbance = np.zeros_like(time)
    disturbance[(time >= 3.6) & (time < 3.9)] = -4.0

    def simulate(use_feedforward: bool) -> tuple[FloatArray, FloatArray]:
        state = np.zeros((3, time.size))
        control = np.zeros(time.size)
        for index in range(time.size - 1):
            target = np.array(
                [reference[index], reference_velocity[index], reference_acceleration[index]]
            )
            nominal = feedforward[index] if use_feedforward else 0.0
            control[index] = np.clip(
                nominal + gains @ (target - state[:, index]),
                -25.0,
                25.0,
            )
            total_input = control[index] + disturbance[index]
            applied_input = float(total_input)

            def rhs(
                _time: float,
                current_state: FloatArray,
                applied_input_value: float = applied_input,
            ) -> FloatArray:
                return np.array(
                    [
                        current_state[1],
                        current_state[2],
                        -a1 * current_state[1]
                        - a2 * current_state[2]
                        + input_gain * applied_input_value,
                    ]
                )

            state[:, index + 1] = rk4_step(rhs, time[index], state[:, index], dt)
        control[-1] = control[-2]
        return state[0], control

    feedback_response, feedback_control = simulate(False)
    two_dof_response, two_dof_control = simulate(True)
    return RotaryResult(
        time=time,
        reference=reference,
        feedback_only_response=feedback_response,
        two_dof_response=two_dof_response,
        feedback_only_control=feedback_control,
        two_dof_control=two_dof_control,
        disturbance=disturbance,
        feedback_only_rmse=rms(reference - feedback_response),
        two_dof_rmse=rms(reference - two_dof_response),
        maximum_two_dof_control=peak_abs(two_dof_control),
    )


@dataclass(frozen=True)
class SuspensionResult:
    time: FloatArray
    road: FloatArray
    passive_body: FloatArray
    active_body: FloatArray
    passive_acceleration: FloatArray
    active_acceleration: FloatArray
    passive_suspension_travel: FloatArray
    active_suspension_travel: FloatArray
    passive_tire_deflection: FloatArray
    active_tire_deflection: FloatArray
    active_force: FloatArray
    gain: FloatArray
    passive_acceleration_rms: float
    active_acceleration_rms: float
    peak_active_force: float


def active_suspension_simulation(dt: float = 0.0005) -> SuspensionResult:
    """Simulate passive and LQR-controlled quarter-car responses."""
    validate_time_step(dt)
    sprung_mass, unsprung_mass = 2.45, 1.40
    suspension_stiffness, tire_stiffness = 900.0, 2500.0
    suspension_damping, tire_damping = 12.0, 10.0

    a = np.array(
        [
            [0.0, 1.0, 0.0, 0.0],
            [
                -suspension_stiffness / sprung_mass,
                -suspension_damping / sprung_mass,
                suspension_stiffness / sprung_mass,
                suspension_damping / sprung_mass,
            ],
            [0.0, 0.0, 0.0, 1.0],
            [
                suspension_stiffness / unsprung_mass,
                suspension_damping / unsprung_mass,
                -(suspension_stiffness + tire_stiffness) / unsprung_mass,
                -(suspension_damping + tire_damping) / unsprung_mass,
            ],
        ]
    )
    control_input = np.array([[0.0], [1.0 / sprung_mass], [0.0], [-1.0 / unsprung_mass]])
    road_input = np.array(
        [
            [0.0, 0.0],
            [0.0, 0.0],
            [0.0, 0.0],
            [tire_stiffness / unsprung_mass, tire_damping / unsprung_mass],
        ]
    )

    q = np.diag([1000.0, 50.0, 500.0, 20.0])
    r = np.array([[0.1]])
    riccati = solve_continuous_are(a, control_input, q, r)
    gain = np.linalg.solve(r, control_input.T @ riccati)

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

    for index in range(time.size - 1):
        disturbance = np.array([road[index], road_velocity[index]])
        force = float(np.clip(-(gain @ active[:, index]).item(), -50.0, 50.0))
        active_force[index] = force
        road_disturbance = disturbance.copy()
        applied_force = force

        def passive_rhs(
            _time: float,
            state: FloatArray,
            road_disturbance_value: FloatArray = road_disturbance,
        ) -> FloatArray:
            return a @ state + road_input @ road_disturbance_value

        def active_rhs(
            _time: float,
            state: FloatArray,
            applied_force_value: float = applied_force,
            road_disturbance_value: FloatArray = road_disturbance,
        ) -> FloatArray:
            return (
                a @ state
                + control_input[:, 0] * applied_force_value
                + road_input @ road_disturbance_value
            )

        passive[:, index + 1] = rk4_step(passive_rhs, time[index], passive[:, index], dt)
        active[:, index + 1] = rk4_step(active_rhs, time[index], active[:, index], dt)
        passive_acceleration[index] = passive_rhs(time[index], passive[:, index])[1]
        active_acceleration[index] = active_rhs(time[index], active[:, index])[1]

    passive_acceleration[-1] = passive_acceleration[-2]
    active_acceleration[-1] = active_acceleration[-2]
    active_force[-1] = active_force[-2]
    passive_travel = passive[0] - passive[2]
    active_travel = active[0] - active[2]
    passive_tire = passive[2] - road
    active_tire = active[2] - road

    return SuspensionResult(
        time=time,
        road=road,
        passive_body=passive[0],
        active_body=active[0],
        passive_acceleration=passive_acceleration,
        active_acceleration=active_acceleration,
        passive_suspension_travel=passive_travel,
        active_suspension_travel=active_travel,
        passive_tire_deflection=passive_tire,
        active_tire_deflection=active_tire,
        active_force=active_force,
        gain=gain.ravel(),
        passive_acceleration_rms=rms(passive_acceleration),
        active_acceleration_rms=rms(active_acceleration),
        peak_active_force=peak_abs(active_force),
    )


@dataclass(frozen=True)
class MaglevResult:
    time: FloatArray
    reference: FloatArray
    linear_position: FloatArray
    nonlinear_position: FloatArray
    nonlinear_current: FloatArray
    total_voltage: FloatArray
    equilibrium_current: float
    equilibrium_voltage: float
    open_loop_poles: FloatArray
    closed_loop_poles: FloatArray
    linear_nonlinear_rmse: float
    maximum_voltage: float


def magnetic_levitation_simulation(dt: float = 0.0001) -> MaglevResult:
    """Compare the local linear model with the nonlinear maglev plant."""
    validate_time_step(dt)
    mass, gravity, gap, magnetic_coefficient = 0.068, 9.81, 0.014, 6.53e-5
    resistance, inductance = 11.0, 0.4125
    equilibrium_current = np.sqrt(2.0 * mass * gravity * gap**2 / magnetic_coefficient)
    equilibrium_voltage = resistance * equilibrium_current

    position_gradient = 2.0 * gravity / gap
    current_gradient = -magnetic_coefficient * equilibrium_current / (mass * gap**2)
    a = np.array(
        [
            [0.0, 1.0, 0.0],
            [position_gradient, 0.0, current_gradient],
            [0.0, 0.0, -resistance / inductance],
        ]
    )
    b = np.array([[0.0], [0.0], [1.0 / inductance]])
    gain = np.array([[-6316.60910998, -168.35876027, 26.125]])
    prefilter = -1009.79192166

    time = np.arange(0.0, 0.5 + dt, dt)
    reference = np.zeros_like(time)
    reference[time >= 0.05] = 0.0005
    linear_state = np.zeros((3, time.size))
    nonlinear_state = np.zeros((3, time.size))
    nonlinear_state[:, 0] = np.array([gap, 0.0, equilibrium_current])
    total_voltage = np.zeros(time.size)

    for index in range(time.size - 1):
        linear_voltage_deviation = float(
            (-gain @ linear_state[:, index]).item() + prefilter * reference[index]
        )
        linear_voltage_deviation = float(
            np.clip(linear_voltage_deviation, -equilibrium_voltage, 30.0 - equilibrium_voltage)
        )

        nonlinear_deviation = nonlinear_state[:, index] - np.array([gap, 0.0, equilibrium_current])
        nonlinear_voltage_deviation = float(
            (-gain @ nonlinear_deviation).item() + prefilter * reference[index]
        )
        total_voltage[index] = np.clip(
            equilibrium_voltage + nonlinear_voltage_deviation,
            0.0,
            30.0,
        )

        def linear_rhs(
            _time: float,
            state: FloatArray,
            voltage_deviation: float = linear_voltage_deviation,
        ) -> FloatArray:
            return a @ state + b[:, 0] * voltage_deviation

        def nonlinear_rhs(
            _time: float,
            state: FloatArray,
            applied_voltage: float = float(total_voltage[index]),
        ) -> FloatArray:
            current_gap = max(state[0], 0.004)
            current = max(state[2], 0.0)
            magnetic_force = magnetic_coefficient * current**2 / (2.0 * current_gap**2)
            return np.array(
                [
                    state[1],
                    gravity - magnetic_force / mass,
                    (applied_voltage - resistance * state[2]) / inductance,
                ]
            )

        linear_state[:, index + 1] = rk4_step(linear_rhs, time[index], linear_state[:, index], dt)
        nonlinear_state[:, index + 1] = rk4_step(
            nonlinear_rhs, time[index], nonlinear_state[:, index], dt
        )

    total_voltage[-1] = total_voltage[-2]
    nonlinear_deviation = nonlinear_state[0] - gap
    return MaglevResult(
        time=time,
        reference=reference,
        linear_position=linear_state[0],
        nonlinear_position=nonlinear_deviation,
        nonlinear_current=nonlinear_state[2],
        total_voltage=total_voltage,
        equilibrium_current=float(equilibrium_current),
        equilibrium_voltage=float(equilibrium_voltage),
        open_loop_poles=np.linalg.eigvals(a),
        closed_loop_poles=np.linalg.eigvals(a - b @ gain),
        linear_nonlinear_rmse=rms(linear_state[0] - nonlinear_deviation),
        maximum_voltage=float(np.max(total_voltage)),
    )


@dataclass(frozen=True)
class TankCase:
    h1: FloatArray
    h2: FloatArray
    saturated_control: FloatArray
    integral_state: FloatArray
    recovery_time: float


@dataclass(frozen=True)
class TwoTankResult:
    time: FloatArray
    reference: FloatArray
    disturbance: FloatArray
    initial_h1: float
    initial_h2: float
    maximum_steady_state_level: float
    no_anti_windup: TankCase
    anti_windup: TankCase


def two_tank_simulation(dt: float = 0.05) -> TwoTankResult:
    """Compare PI control with and without back-calculation anti-windup."""
    validate_time_step(dt)
    area_1 = area_2 = 50.0
    outlet_12, outlet_2, gravity = 0.20, 0.18, 981.0
    kp, ki, kaw = 5.0, 0.05, 0.30
    u_min, u_max = 0.0, 45.0

    initial_h2 = 10.0
    steady_flow = outlet_2 * np.sqrt(2.0 * gravity * initial_h2)
    initial_h1 = initial_h2 + (steady_flow / outlet_12) ** 2 / (2.0 * gravity)
    initial_integral = steady_flow / ki
    maximum_steady_state_level = (u_max / outlet_2) ** 2 / (2.0 * gravity)

    time = np.arange(0.0, 700.0 + dt, dt)
    reference = np.full_like(time, initial_h2)
    reference[(time >= 100.0) & (time < 160.0)] = 35.0
    disturbance = np.zeros_like(time)
    disturbance[(time >= 520.0) & (time < 580.0)] = -5.0

    def simulate(use_anti_windup: bool) -> TankCase:
        state = np.zeros((2, time.size))
        state[:, 0] = np.array([initial_h1, initial_h2])
        control = np.zeros_like(time)
        integral = np.zeros_like(time)
        integral[0] = initial_integral

        for index in range(time.size - 1):
            error = reference[index] - state[1, index]
            unsaturated = kp * error + ki * integral[index]
            control[index] = np.clip(unsaturated, u_min, u_max)
            correction = kaw * (control[index] - unsaturated) if use_anti_windup else 0.0
            integral[index + 1] = integral[index] + dt * (error + correction)
            inlet = control[index] + disturbance[index]

            def rhs(
                _time: float,
                current_state: FloatArray,
                applied_inlet: float = float(inlet),
            ) -> FloatArray:
                h1, h2 = np.maximum(current_state, 0.0)
                flow_12 = outlet_12 * np.sqrt(2.0 * gravity * max(h1 - h2, 0.0))
                flow_out = outlet_2 * np.sqrt(2.0 * gravity * h2)
                return np.array(
                    [
                        (applied_inlet - flow_12) / area_1,
                        (flow_12 - flow_out) / area_2,
                    ]
                )

            next_state = rk4_step(rhs, time[index], state[:, index], dt)
            state[:, index + 1] = np.maximum(next_state, 0.0)
        control[-1] = control[-2]
        recovery = sustained_entry_time(
            time,
            state[1] - initial_h2,
            tolerance=1.0,
            start_time=160.0,
            hold_time=20.0,
            end_time=480.0,
        )
        return TankCase(state[0], state[1], control, integral, recovery)

    return TwoTankResult(
        time=time,
        reference=reference,
        disturbance=disturbance,
        initial_h1=float(initial_h1),
        initial_h2=initial_h2,
        maximum_steady_state_level=float(maximum_steady_state_level),
        no_anti_windup=simulate(False),
        anti_windup=simulate(True),
    )


def repository_root() -> Path:
    return Path(__file__).resolve().parents[1]
