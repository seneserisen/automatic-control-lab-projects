import numpy as np
import pytest

from validation.maglev_observer import (
    maglev_design,
    maglev_observer_convergence,
    magnetic_levitation_observer_simulation,
)


def test_maglev_observer_design_is_stable_and_observable() -> None:
    design = maglev_design()
    assert design.observability_rank == 3
    assert np.max(np.real(design.controller_poles)) < 0.0
    assert np.max(np.real(design.observer_poles)) < 0.0


def test_observer_tracks_reference_with_sensor_noise() -> None:
    result = magnetic_levitation_observer_simulation()
    assert abs(result.observer_position[-1] - 0.0005) < 1e-5
    assert result.position_estimation_rmse < 2e-6
    assert result.velocity_estimation_rmse < 1e-4
    assert result.current_estimation_rmse < 5e-4
    assert result.maximum_voltage <= 30.0


def test_observer_simulation_is_deterministic_for_fixed_seed() -> None:
    first = magnetic_levitation_observer_simulation(seed=7)
    second = magnetic_levitation_observer_simulation(seed=7)
    assert np.array_equal(first.observer_position, second.observer_position)
    assert np.array_equal(first.estimated_velocity, second.estimated_velocity)


def test_observer_rejects_negative_noise_standard_deviation() -> None:
    with pytest.raises(ValueError, match="non-negative"):
        magnetic_levitation_observer_simulation(gap_noise_std=-1.0)


def test_observer_fixed_step_solution_converges() -> None:
    points = maglev_observer_convergence()
    fine = points[-1]
    medium = points[-2]
    assert abs(medium.final_position - fine.final_position) < 1e-8
    assert abs(medium.maximum_voltage - fine.maximum_voltage) < 1e-3
