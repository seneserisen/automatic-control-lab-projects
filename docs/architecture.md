# Repository architecture

## Design goals

The repository separates six concerns:

1. **Engineering experiments** — MATLAB demonstrations that explain modelling and control decisions.
2. **Reusable numerical utilities** — shared MATLAB functions for integration, saturation, tracking metrics and recovery metrics.
3. **Modular experiment functions** — configuration, plant model, controller design, simulation, metrics and plotting can be tested independently.
4. **Independent reference validation** — Python models implement the same equations for regression checks and generated evidence.
5. **Deployment-oriented runtime code** — portable C implementations express selected control algorithms using fixed-size memory and deterministic cyclic execution.
6. **Published evidence** — generated figures, quantitative results, source-document reconstruction notes and requirements-to-test traceability.

## Source-document transformation

The uploaded reports, MATLAB scripts, Simulink models, preparation questions, experiment instructions and result discussions are treated as engineering source specifications.

They are transformed into original public artifacts rather than uploaded unchanged. The transformation policy is documented in `docs/source-reconstruction-map.md`.

## MATLAB experiment layer

Each folder in `projects/` contains:

- a self-contained project README;
- one executable demonstration entry point;
- project functions where deeper testing is valuable;
- generated figures.

The magnetic-levitation project is the first fully modular experiment:

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

This avoids duplicating integration and metric code across experiments.

## Output-feedback observer architecture

The magnetic-levitation controller measures gap and coil current. Ball velocity is estimated with a Luenberger observer:

```text
reference ──► state-feedback controller ──► voltage saturation ──► nonlinear plant
                    ▲                                                │
                    │                                                ▼
                    └──────── estimated state ◄── observer ◄── gap/current sensors
```

The nonlinear plant is controlled using the estimated local-deviation state. A separate full-state nonlinear simulation provides a comparison baseline.

## Python validation layer

`validation/reference_models.py` contains deterministic implementations of the five original experiment comparisons.

`validation/maglev_observer.py` contains the output-feedback observer experiment and fixed-step convergence study.

`validation/numerics.py` contains reusable numerical utilities. `validation/report.py` generates `docs/results-summary.md`. CI fails when the committed report no longer matches the executable reference models.

Python is deliberately independent from the MATLAB implementation. That independence helps detect repeated modelling or implementation mistakes.

## Portable C runtime layer

The `c/` directory contains deployment-oriented implementations for algorithms that would plausibly run cyclically on an embedded controller.

The magnetic-levitation C runtime contains:

- public C99 configuration and metrics API;
- nonlinear plant equations;
- precomputed controller and observer gains;
- fixed-step RK4 integration;
- deterministic sensor-noise support;
- voltage saturation;
- online metrics;
- no dynamic allocation;
- CMake and CTest configuration.

C is not used for every laboratory document automatically. It is added when deterministic execution, reusable control logic, embedded deployment, or hardware-in-the-loop validation provides clear engineering value.

## Three-language continuous integration

Three workflows are used:

### Python validation

- Ruff lint and formatting checks
- Python 3.10, 3.11 and 3.12
- Numerical and physical-behaviour tests
- Generated-results freshness checks

### MATLAB validation

- Fixed MATLAB R2024b runtime
- Direct `matlab.unittest` execution
- Strict warning handling
- JUnit test-result artifact
- Shared utilities and magnetic-levitation observer functions on the MATLAB path

### C runtime validation

- GCC and Clang matrix
- C99 without compiler extensions
- `-Wall -Wextra -Wpedantic -Werror`
- CMake build
- CTest runtime checks
- Reference demo execution

The three implementations have different roles:

- MATLAB proves the design and MATLAB runtime behaviour;
- Python provides an independent numerical oracle and automation layer;
- C demonstrates a deterministic deployment-oriented structure.

## Cross-language comparison policy

Noise-free scenarios share numerical tolerances such as reference tracking, voltage limits and step-size convergence.

Noisy trajectories are not required to be bitwise-identical because MATLAB, NumPy and the C runtime use different random-number generators. They must satisfy the same physical and performance requirements.

## Test boundaries

Automated tests verify:

- equilibrium stability, controllability and observability;
- local linearisation accuracy;
- 2-DOF tracking improvement;
- active-suspension acceleration reduction;
- nonlinear magnetic-levitation tracking and voltage limits;
- noisy observer-based state estimation;
- deterministic seeded execution within each implementation;
- fixed-step convergence;
- portable C configuration validation;
- GCC and Clang warning-free compilation;
- physical reachability of the two-tank reference;
- anti-windup recovery improvement;
- RK4 accuracy and input validation;
- sustained recovery-time and tracking-metric logic.

The tests do not establish hardware safety, real-time schedulability, fixed-point correctness, MISRA-C compliance, or robustness outside the committed parameter ranges.

## Numerical integration decision

Classical fourth-order Runge-Kutta is used for fixed-step simulations. RK4 reduces integration error substantially without hiding the state equations behind a black-box solver. The magnetic-levitation observer includes a no-noise step-size study so the selected 0.1 ms step is supported by measured convergence evidence.

## Reproducibility

- Plant parameters, controller gains and observer gains are visible in code.
- Sensor noise is deterministic within each implementation through a committed seed.
- Convergence studies disable noise to isolate integration error.
- Actuator limits are explicit.
- Published metrics are generated rather than typed manually.
- Requirements are linked to automated tests in `docs/verification-matrix.md`.
- Historical source material is mapped to public artifacts in `docs/source-reconstruction-map.md`.
