import numpy as np

from validation.reference_models import (
    active_suspension_simulation,
    magnetic_levitation_simulation,
    nonlinear_linearisation,
    rotary_arm_simulation,
    two_tank_simulation,
)


def test_nonlinear_equilibria_have_expected_local_stability() -> None:
    result = nonlinear_linearisation()
    assert np.max(np.real(result.stable_eigenvalues)) < 0
    assert np.max(np.real(result.unstable_eigenvalues)) > 0
    assert np.allclose(np.sort(result.closed_loop_eigenvalues), [-3.0, -2.0])


def test_rotary_arm_tracks_reference_within_tolerance() -> None:
    result = rotary_arm_simulation()
    assert abs(result.response[-1] - 1.0) < 1e-3
    assert np.max(np.abs(result.reference - result.response)) < 1e-3
    assert np.max(np.abs(result.control)) <= 25.0 + 1e-12


def test_active_suspension_reduces_peak_body_acceleration() -> None:
    result = active_suspension_simulation()
    passive_peak = np.max(np.abs(result.passive_acceleration))
    active_peak = np.max(np.abs(result.active_acceleration))
    assert active_peak < passive_peak


def test_maglev_equilibrium_and_closed_loop_stability() -> None:
    result = magnetic_levitation_simulation()
    assert abs(result.equilibrium_current - 2.0011) < 5e-3
    assert np.max(np.real(result.open_loop_poles)) > 0
    assert np.max(np.real(result.closed_loop_poles)) < 0
    assert abs(result.position[-1] - 0.001) < 2e-5


def test_anti_windup_recovers_faster_after_saturation() -> None:
    result = two_tank_simulation()
    mask = result.time >= 300.0
    time = result.time[mask]
    no_aw_error = np.abs(result.no_anti_windup.h2[mask] - 10.0)
    aw_error = np.abs(result.anti_windup.h2[mask] - 10.0)
    no_aw_recovery = time[np.where(no_aw_error < 1.0)[0][0]]
    aw_recovery = time[np.where(aw_error < 1.0)[0][0]]
    assert aw_recovery < no_aw_recovery
