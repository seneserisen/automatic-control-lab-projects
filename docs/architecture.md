# Repository architecture

## Overview

The repository separates modelling, experiment orchestration, validation, and portable runtime code.

```text
projects/                  MATLAB experiments and project-specific functions
matlab/+control_lab/       Shared MATLAB numerical and metrics utilities
matlab/tests/              Direct MATLAB unit tests
validation/                Numerical reference models and generated reports
tests/                     Python regression and behaviour tests
c/                         Portable C99 controller and observer runtime
c/tests/                   CTest-based runtime tests
docs/                      Architecture, results, and requirements traceability
.github/workflows/         MATLAB, Python, and C CI workflows
```

## MATLAB experiment layer

Each project folder contains:

- an executable demonstration;
- project-specific model and controller functions where required;
- quantitative metrics;
- generated figures;
- assumptions and limitations.

The magnetic-levitation project is fully modular:

```text
maglev_configuration.m
maglev_linear_model.m
design_maglev_observer.m
maglev_nonlinear_dynamics.m
simulate_maglev_observer.m
calculate_maglev_observer_metrics.m
maglev_convergence_study.m
plot_maglev_observer_results.m
magnetic_levitation_demo.m
```

The demo coordinates the workflow; the functions contain the testable engineering logic.

## Shared MATLAB package

The `control_lab` package provides:

- `control_lab.rk4_step`
- `control_lab.rms_value`
- `control_lab.saturate`
- `control_lab.sustained_entry_time`
- `control_lab.tracking_metrics`

## Magnetic-levitation observer

The magnetic-levitation controller measures gap and coil current. Ball velocity is estimated with a Luenberger observer:

```text
reference ──► state-feedback controller ──► voltage saturation ──► nonlinear plant
                    ▲                                                │
                    │                                                ▼
                    └──────── estimated state ◄── observer ◄── gap/current sensors
```

The nonlinear plant is controlled using the estimated local-deviation state. A separate full-state nonlinear simulation provides the comparison baseline.

## Numerical validation

`validation/reference_models.py` contains deterministic reference implementations for the five experiments.

`validation/maglev_observer.py` contains the magnetic-levitation observer and fixed-step convergence study.

`validation/report.py` generates `docs/results-summary.md`. CI fails when the committed report no longer matches the executable models.

## Portable C runtime

The `c/` directory contains the magnetic-levitation observer runtime:

- public C99 configuration and metrics API;
- nonlinear plant equations;
- precomputed controller and observer gains;
- fixed-step RK4 integration;
- deterministic sensor-noise support;
- voltage saturation;
- online metrics;
- fixed-size memory without dynamic allocation;
- CMake and CTest configuration.

## Continuous integration

### Python

- Ruff lint and formatting checks
- Python 3.10, 3.11, and 3.12
- numerical and physical-behaviour tests
- generated-results freshness checks

### MATLAB

- MATLAB R2024b
- direct `matlab.unittest` execution
- strict warning handling
- JUnit test-result artifact

### C

- GCC and Clang
- C99 without compiler extensions
- `-Wall -Wextra -Wpedantic -Werror`
- CMake build
- CTest runtime checks
- reference demo execution

## Test boundaries

Automated tests verify:

- stability, controllability, and observability;
- local linearisation accuracy;
- tracking improvement and disturbance rejection;
- active-suspension acceleration reduction;
- magnetic-levitation tracking and voltage limits;
- observer-based state estimation;
- deterministic seeded execution;
- fixed-step convergence;
- C configuration validation;
- anti-windup recovery;
- RK4 and metric utilities.

They do not establish hardware safety, real-time schedulability, fixed-point correctness, MISRA-C compliance, or robustness outside the tested parameter ranges.

## Reproducibility

- Parameters, controller gains, and observer gains are visible in code.
- Random seeds are fixed for published experiments.
- Convergence studies disable noise to isolate integration error.
- Actuator limits are explicit.
- Metrics and reports are generated from executable models.
- Requirements are linked to automated tests in `docs/verification-matrix.md`.
