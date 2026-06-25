# Automatic Control Laboratory Projects

[![Validate control portfolio](https://github.com/seneserisen/automatic-control-lab-projects/actions/workflows/validate.yml/badge.svg)](https://github.com/seneserisen/automatic-control-lab-projects/actions/workflows/validate.yml)
[![Validate MATLAB experiments](https://github.com/seneserisen/automatic-control-lab-projects/actions/workflows/matlab.yml/badge.svg)](https://github.com/seneserisen/automatic-control-lab-projects/actions/workflows/matlab.yml)
[![Validate C control runtime](https://github.com/seneserisen/automatic-control-lab-projects/actions/workflows/c.yml/badge.svg)](https://github.com/seneserisen/automatic-control-lab-projects/actions/workflows/c.yml)

A control-engineering portfolio covering nonlinear systems, state-space control, LQR, observers, actuator saturation, anti-windup, numerical validation, and portable C implementation.

## Projects

| Project | Main methods | Published result |
|---|---|---|
| [Nonlinear control loops](projects/nonlinear-control-loops/) | Jacobian linearisation, equilibrium stability, controllability, local state feedback | Linearisation error increases away from the operating point |
| [Elastically mounted rotary arm](projects/rotary-arm/) | Fifth-order trajectory, feedback, feedforward, 2-DOF control, load disturbance | 2-DOF tracking RMSE is about 70% lower |
| [Quarter-car active suspension](projects/active-suspension/) | State-space modelling, LQR, road disturbance, actuator saturation | RMS body acceleration is reduced by about 35% |
| [Magnetic levitation](projects/magnetic-levitation/) | Nonlinear plant, pole placement, Luenberger observer, sensor noise, convergence study, portable C runtime | Position-estimation RMSE is below 0.001 mm |
| [Two-tank process](projects/two-tank-system/) | Nonlinear hydraulics, PI control, saturation, anti-windup | Recovery improves from about 429 s to 312 s |

Detailed numerical results are generated from executable models and published in [docs/results-summary.md](docs/results-summary.md).

## Preview results

| Active suspension | Magnetic levitation | Two-tank anti-windup |
|---|---|---|
| ![Active suspension](projects/active-suspension/figures/active_suspension_results.svg) | ![Magnetic levitation](projects/magnetic-levitation/figures/magnetic_levitation_observer_results.svg) | ![Two-tank](projects/two-tank-system/figures/two_tank_results.svg) |

## Technical coverage

- Nonlinear differential-equation modelling
- Operating-point and Jacobian linearisation
- State-space modelling, controllability, and observability
- Pole placement and eigenvalue analysis
- Luenberger observers and output-feedback control
- LQR control and disturbance rejection
- P and PI control with back-calculation anti-windup
- Actuator saturation and control-effort analysis
- Fourth-order Runge-Kutta integration and convergence studies
- Deterministic sensor-noise scenarios
- Portable C99 runtime with fixed-size memory
- CMake, CTest, GCC, Clang, and strict compiler warnings
- Direct MATLAB and multi-version Python CI

## Repository structure

```text
projects/                  MATLAB experiments and project-specific functions
matlab/+control_lab/       Shared MATLAB numerical and metrics utilities
matlab/tests/              Direct matlab.unittest verification
validation/                Independent numerical reference models and reports
tests/                     Python numerical and behaviour tests
c/                         Portable C99 controller and observer runtime
c/tests/                   CTest-based runtime verification
docs/                      Architecture, results, and requirements traceability
.github/workflows/         MATLAB, Python, and C CI workflows
```

See [docs/architecture.md](docs/architecture.md) and [docs/verification-matrix.md](docs/verification-matrix.md).

## Quick start

### MATLAB

```matlab
run_all
```

Run the magnetic-levitation observer experiment:

```matlab
run('projects/magnetic-levitation/magnetic_levitation_demo.m')
```

Run MATLAB tests:

```matlab
results = runtests('matlab/tests', 'IncludeSubfolders', true);
assertSuccess(results);
```

### Portable C runtime

```bash
cmake -S c -B build/c -DCMAKE_BUILD_TYPE=Release
cmake --build build/c --parallel
ctest --test-dir build/c --output-on-failure
./build/c/maglev_observer_demo
```

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

## Validation

- MATLAB R2024b executes direct `matlab.unittest` tests.
- Python 3.10–3.12 executes numerical regression and report-freshness checks.
- GCC and Clang compile the C99 runtime with warnings treated as errors.
- CTest verifies configuration handling, tracking, saturation, determinism, and convergence.
- Requirements and automated evidence are linked in [docs/verification-matrix.md](docs/verification-matrix.md).

## Limitations

These are simulation and software-validation projects, not production controllers. They do not establish:

- hardware-in-the-loop performance;
- real-time scheduling guarantees;
- fixed-point numerical behaviour;
- MISRA-C compliance;
- robust stability outside the tested parameter ranges;
- functional-safety certification.

Each project README documents its specific assumptions and validation boundaries.

## Attribution

The original laboratory exercises were completed in a four-person academic team. This repository contains independently structured portfolio implementations and documentation. See [NOTICE.md](NOTICE.md).
