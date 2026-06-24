# Automatic Control Laboratory Projects

[![Validate control portfolio](https://github.com/seneserisen/automatic-control-lab-projects/actions/workflows/validate.yml/badge.svg)](https://github.com/seneserisen/automatic-control-lab-projects/actions/workflows/validate.yml)

A curated control-engineering portfolio with five reproducible MATLAB experiments and a licence-free Python validation layer.

The repository reconstructs the main engineering ideas from a four-person **Automatic Control I** laboratory portfolio at FAU Erlangen-Nürnberg. The public code was rebuilt from first principles rather than copied from raw team submissions or university material.

## Why this repository exists

The objective is not to present five isolated scripts. It is to demonstrate a consistent control-engineering workflow:

1. define the physical or nonlinear model;
2. identify an operating point or state-space representation;
3. analyse stability and controllability;
4. design a controller;
5. simulate disturbances and actuator limits;
6. calculate repeatable performance metrics;
7. document what the model can and cannot prove.

## Projects

| Project | Main methods | Published result |
|---|---|---|
| [Nonlinear control loops](projects/nonlinear-control-loops/) | Jacobian linearisation, equilibrium stability, controllability, local state feedback | Linearisation error increases away from the operating point |
| [Elastically mounted rotary arm](projects/rotary-arm/) | Fifth-order trajectory, feedback, feedforward, 2-DOF control, load disturbance | 2-DOF tracking RMSE is about 70% lower |
| [Quarter-car active suspension](projects/active-suspension/) | State-space modelling, LQR, road disturbance, actuator saturation | RMS body acceleration is reduced by about 35% |
| [Magnetic levitation](projects/magnetic-levitation/) | Nonlinear equilibrium, linearisation, pole placement, nonlinear validation | Linear/nonlinear gap RMSE is below 0.001 mm |
| [Two-tank process](projects/two-tank-system/) | Nonlinear hydraulics, PI control, saturation, anti-windup | Recovery improves from about 429 s to 312 s |

Detailed values are generated from the reference simulations and committed in [docs/results-summary.md](docs/results-summary.md).

## Preview results

| Active suspension | Magnetic levitation | Two-tank anti-windup |
|---|---|---|
| ![Active suspension](projects/active-suspension/figures/active_suspension_results.svg) | ![Magnetic levitation](projects/magnetic-levitation/figures/magnetic_levitation_results.svg) | ![Two-tank](projects/two-tank-system/figures/two_tank_results.svg) |

## Repository architecture

```text
projects/                  MATLAB experiments and project explanations
matlab/+control_lab/       Shared RK4, saturation, RMS and recovery utilities
validation/                Python reference models and generated metrics
tests/                     Numerical and physical-behaviour tests
docs/                      Architecture, assumptions, results and validation notes
.github/workflows/         Python 3.10–3.12 linting and test matrix
```

See [docs/architecture.md](docs/architecture.md) for the design rationale.

## Technical coverage

- Nonlinear differential-equation modelling
- Operating-point and Jacobian linearisation
- State-space modelling and controllability
- Pole and eigenvalue analysis
- Frequency-response reasoning
- P, PI, state-feedback, pole-placement and LQR concepts
- Smooth reference trajectories and two-degree-of-freedom control
- Disturbance rejection
- Actuator saturation and back-calculation anti-windup
- Fourth-order Runge-Kutta simulation
- Reproducible performance metrics and CI

## Quick start

### MATLAB

From the repository root:

```matlab
run_all
```

Or run one project:

```matlab
run('projects/magnetic-levitation/magnetic_levitation_demo.m')
```

Recommended environment:

- MATLAB R2021b or later
- Control System Toolbox is optional; verified fallback gains are included
- No `.slx` files are required for the clean-room demonstrations

Complete [docs/manual-validation-checklist.md](docs/manual-validation-checklist.md) before describing the repository as MATLAB-validated.

### Python validation

```bash
python -m venv .venv
source .venv/bin/activate  # Windows: .venv\Scripts\activate
pip install -r requirements.txt
ruff check .
ruff format --check .
pytest -q
python -m validation.generate_reference_figures --check-only
python -m validation.report --check
```

To regenerate the SVG previews and results summary:

```bash
python -m validation.generate_reference_figures
python -m validation.report
```

## Validation strategy

The MATLAB scripts are the main portfolio experiments. The Python layer independently mirrors the equations to provide:

- licence-free CI on standard GitHub runners;
- numerical regression tests;
- deterministic result figures;
- checks for stability, actuator limits and physically meaningful scenarios;
- a generated results report that CI verifies is current.

This does not replace MATLAB execution. It provides a second implementation that makes the published claims auditable.

## Limitations

These are educational models, not production controllers. They do not establish:

- hardware-in-the-loop performance;
- real-time timing guarantees;
- sensor or actuator reliability;
- robust stability across uncertain plant parameters;
- functional-safety compliance;
- equivalence to the original team submission.

Each project README states its specific assumptions and missing validation.

## Academic attribution

The historical laboratory exercises were completed in a four-person team. The repository contains new clean-room portfolio reconstructions. See [NOTICE.md](NOTICE.md) and [docs/team-contributions-template.md](docs/team-contributions-template.md).
