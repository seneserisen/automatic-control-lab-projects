# Repository architecture

## Design goals

The repository separates four concerns:

1. **Engineering experiments** — readable MATLAB scripts showing the modelling and control workflow.
2. **Reusable numerical utilities** — shared MATLAB functions for integration, saturation and metrics.
3. **Independent validation** — Python models implementing the same equations for automated checks.
4. **Published evidence** — generated SVG figures and a version-controlled metrics report.

## MATLAB experiment layer

Each folder in `projects/` contains:

- a self-contained project README;
- one executable demonstration script;
- generated figures.

The scripts call utilities in the `control_lab` MATLAB package:

- `control_lab.rk4_step`
- `control_lab.rms_value`
- `control_lab.saturate`
- `control_lab.sustained_entry_time`

This avoids duplicating numerical integration and metric logic across experiments.

## Python validation layer

`validation/reference_models.py` contains deterministic implementations of the five experiments. The Python models are not intended to replace MATLAB. They exist so the repository can be checked on a licence-free CI runner.

`validation/numerics.py` contains reusable numerical and metric functions. `validation/report.py` generates `docs/results-summary.md` from the current models. CI fails when the committed report no longer matches the code.

## Test boundaries

The automated tests verify:

- equilibrium stability and controllability;
- local linearisation accuracy;
- 2-DOF tracking improvement;
- active-suspension acceleration reduction;
- nonlinear magnetic-levitation tracking and voltage limits;
- physical reachability of the two-tank reference;
- anti-windup recovery improvement;
- RK4 accuracy and input validation;
- sustained recovery-time logic.

The tests do not verify MATLAB syntax or MATLAB-specific plotting. That remains part of the manual validation checklist.

## Numerical integration decision

Forward Euler was replaced with classical fourth-order Runge-Kutta for the fixed-step simulations. RK4 reduces integration error substantially without hiding the state equations behind a black-box solver. The nonlinear control-loop experiment still uses `ode45` because its main purpose is comparison with MATLAB's standard adaptive integration workflow.

## Reproducibility

- No random inputs are used.
- All plant parameters and controller gains are visible in code.
- Actuator limits are explicit.
- Published metrics are generated rather than typed manually.
- CI runs on Python 3.10, 3.11 and 3.12.
