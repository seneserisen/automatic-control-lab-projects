import numpy as np
import pytest

from validation.numerics import rk4_step, sustained_entry_time
from validation.reference_models import (
    active_suspension_simulation,
    magnetic_levitation_simulation,
    nonlinear_linearisation,
    rotary_arm_simulation,
    two_tank_simulation,
)


def test_nonlinear_equilibria_and_controllability() -> None:
    result = nonlinear_linearisation()
    assert result.controllability_rank == 2
    assert np.max(np.real(result.stable_eigenvalues)) < 0
    assert np.max(np.real(result.unstable_eigenvalues)) > 0
    assert np.allclose(np.sort(result.closed_loop_eigenvalues), [-3.0, -2.0])


def test_linearisation_error_grows_away_from_operating_point() -> None:
    result = nonlinear_linearisation()
    assert result.near_model_rmse < 1e-4
    assert result.far_model_rmse > 20.0 * result.near_model_rmse


def test_two_degree_of_freedom_rotary_control_improves_tracking() -> None:
    result = rotary_arm_simulation()
    assert abs(result.two_dof_response[-1] - 1.0) < 1e-3
    assert result.two_dof_rmse < result.feedback_only_rmse
    assert result.maximum_two_dof_control <= 25.0 + 1e-12


def test_active_suspension_reduces_rms_body_acceleration() -> None:
    result = active_suspension_simulation()
    assert result.active_acceleration_rms < result.passive_acceleration_rms
    assert result.peak_active_force <= 50.0 + 1e-12
    assert np.all(np.isfinite(result.active_suspension_travel))
    assert np.all(np.isfinite(result.active_tire_deflection))


def test_maglev_nonlinear_plant_matches_local_model() -> None:
    result = magnetic_levitation_simulation()
    assert abs(result.equilibrium_current - 2.0011) < 5e-3
    assert np.max(np.real(result.open_loop_poles)) > 0
    assert np.max(np.real(result.closed_loop_poles)) < 0
    assert abs(result.nonlinear_position[-1] - 0.0005) < 2e-5
    assert result.linear_nonlinear_rmse < 2e-6
    assert result.maximum_voltage <= 30.0 + 1e-12
    assert np.min(result.nonlinear_current) >= 0.0


def test_two_tank_reference_is_physically_unreachable() -> None:
    result = two_tank_simulation()
    assert result.maximum_steady_state_level < 35.0
    assert result.initial_h1 > result.initial_h2
    assert result.no_anti_windup.h2[0] == pytest.approx(result.initial_h2)
    assert result.anti_windup.h2[0] == pytest.approx(result.initial_h2)


def test_anti_windup_recovers_faster_after_saturation() -> None:
    result = two_tank_simulation()
    assert np.isfinite(result.no_anti_windup.recovery_time)
    assert np.isfinite(result.anti_windup.recovery_time)
    assert result.anti_windup.recovery_time < result.no_anti_windup.recovery_time
    assert np.max(np.abs(result.anti_windup.integral_state)) < np.max(
        np.abs(result.no_anti_windup.integral_state)
    )


def test_rk4_matches_exponential_decay() -> None:
    dt = 0.05
    state = np.array([1.0])
    time = 0.0
    for _ in range(20):
        state = rk4_step(lambda _t, x: -x, time, state, dt)
        time += dt
    assert state[0] == pytest.approx(np.exp(-1.0), rel=2e-6)


def test_rk4_rejects_invalid_time_step() -> None:
    with pytest.raises(ValueError, match="positive finite"):
        rk4_step(lambda _t, x: x, 0.0, np.array([1.0]), 0.0)


def test_sustained_entry_time_requires_hold_interval() -> None:
    time = np.arange(0.0, 5.1, 0.1)
    error = np.ones_like(time)
    error[(time >= 1.0) & (time < 1.5)] = 0.05
    error[time >= 3.0] = 0.05
    entry = sustained_entry_time(
        time,
        error,
        tolerance=0.1,
        start_time=0.0,
        hold_time=1.0,
    )
    assert entry == pytest.approx(3.0)
