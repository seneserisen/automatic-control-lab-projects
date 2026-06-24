"""Create and verify the published numerical results summary."""

from __future__ import annotations

import argparse

import numpy as np

from validation.reference_models import (
    active_suspension_simulation,
    magnetic_levitation_simulation,
    nonlinear_linearisation,
    repository_root,
    rotary_arm_simulation,
    two_tank_simulation,
)


def render_report() -> str:
    nonlinear = nonlinear_linearisation()
    rotary = rotary_arm_simulation()
    suspension = active_suspension_simulation()
    maglev = magnetic_levitation_simulation()
    tanks = two_tank_simulation()

    acceleration_reduction = 100.0 * (
        1.0 - suspension.active_acceleration_rms / suspension.passive_acceleration_rms
    )
    rotary_improvement = 100.0 * (1.0 - rotary.two_dof_rmse / rotary.feedback_only_rmse)

    return f"""# Reproducible results summary

Generated from the licence-free Python reference models in `validation/`.
These numbers are educational simulation results, not hardware measurements.

## Nonlinear control loops

- Stable-equilibrium maximum pole real part: `{np.max(np.real(nonlinear.stable_eigenvalues)):.4f} 1/s`
- Unstable-equilibrium maximum pole real part: `{np.max(np.real(nonlinear.unstable_eigenvalues)):.4f} 1/s`
- Closed-loop maximum pole real part: `{np.max(np.real(nonlinear.closed_loop_eigenvalues)):.4f} 1/s`
- Controllability rank: `{nonlinear.controllability_rank}`
- Near-operating-point linearisation RMSE: `{nonlinear.near_model_rmse:.6f}`
- Farther-operating-point linearisation RMSE: `{nonlinear.far_model_rmse:.6f}`

The larger error for the farther initial condition demonstrates that linearisation accuracy is local.

## Rotary arm

- Feedback-only tracking RMSE: `{rotary.feedback_only_rmse:.6f} rad`
- Two-degree-of-freedom tracking RMSE: `{rotary.two_dof_rmse:.6f} rad`
- Tracking-RMSE improvement: `{rotary_improvement:.2f}%`
- Peak two-degree-of-freedom control magnitude: `{rotary.maximum_two_dof_control:.3f}`

The two-degree-of-freedom design combines model-based feedforward with feedback disturbance rejection.

## Active suspension

- Passive body-acceleration RMS: `{suspension.passive_acceleration_rms:.4f} m/s²`
- Active body-acceleration RMS: `{suspension.active_acceleration_rms:.4f} m/s²`
- RMS acceleration reduction: `{acceleration_reduction:.2f}%`
- Peak active force: `{suspension.peak_active_force:.3f} N`

The result quantifies ride-comfort improvement while exposing the required actuator effort.

## Magnetic levitation

- Equilibrium current: `{maglev.equilibrium_current:.4f} A`
- Equilibrium voltage: `{maglev.equilibrium_voltage:.4f} V`
- Open-loop maximum pole real part: `{np.max(np.real(maglev.open_loop_poles)):.4f} 1/s`
- Closed-loop maximum pole real part: `{np.max(np.real(maglev.closed_loop_poles)):.4f} 1/s`
- Linear/nonlinear position RMSE: `{maglev.linear_nonlinear_rmse * 1000.0:.6f} mm`
- Maximum applied voltage: `{maglev.maximum_voltage:.4f} V`

The nonlinear comparison verifies that the local controller remains accurate for the selected 0.5 mm reference step.

## Two-tank process

- Initial equilibrium levels: `h1={tanks.initial_h1:.4f} cm`, `h2={tanks.initial_h2:.4f} cm`
- Pump-limited maximum steady-state tank-2 level: `{tanks.maximum_steady_state_level:.4f} cm`
- Deliberately unreachable reference: `35.0000 cm`
- Recovery time without anti-windup: `{tanks.no_anti_windup.recovery_time:.2f} s`
- Recovery time with back-calculation: `{tanks.anti_windup.recovery_time:.2f} s`

The scenario begins at equilibrium and uses a reference above the pump's steady-state capability, so the windup comparison has a clear physical basis.
"""


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()

    output = repository_root() / "docs" / "results-summary.md"
    rendered = render_report()
    if args.check:
        if not output.exists() or output.read_text(encoding="utf-8") != rendered:
            raise SystemExit(
                "docs/results-summary.md is out of date; run python -m validation.report"
            )
        print("Published results summary is current.")
        return

    output.write_text(rendered, encoding="utf-8")
    print(f"Wrote {output}")


if __name__ == "__main__":
    main()
