# Automatic Control Laboratory Projects

[![Validate control portfolio](https://github.com/seneserisen/automatic-control-lab-projects/actions/workflows/validate.yml/badge.svg)](https://github.com/seneserisen/automatic-control-lab-projects/actions/workflows/validate.yml)
[![Validate MATLAB experiments](https://github.com/seneserisen/automatic-control-lab-projects/actions/workflows/matlab.yml/badge.svg)](https://github.com/seneserisen/automatic-control-lab-projects/actions/workflows/matlab.yml)
[![Validate C control runtime](https://github.com/seneserisen/automatic-control-lab-projects/actions/workflows/c.yml/badge.svg)](https://github.com/seneserisen/automatic-control-lab-projects/actions/workflows/c.yml)

A curated control-engineering portfolio with five reproducible MATLAB experiments, direct MATLAB unit tests, an independent Python validation layer, and a portable C control runtime.

The repository reconstructs the main engineering ideas from a four-person **Automatic Control I** laboratory portfolio at FAU Erlangen-Nürnberg. The uploaded reports, MATLAB scripts, Simulink models, preparation material, and result discussions are treated as source specifications. The public code is rebuilt from first principles rather than copied from raw team submissions or university material.

See [docs/source-reconstruction-map.md](docs/source-reconstruction-map.md) for how the historical documents are transformed into public code, tests, results, and explanations.

## Why this repository exists

The objective is not to present five isolated scripts. It is to demonstrate a consistent control-engineering workflow:

1. define the physical or nonlinear model;
2. identify an operating point or state-space representation;
3. analyse stability, controllability and observability;
4. design a controller or state observer;
5. simulate disturbances, noise and actuator limits;
6. calculate repeatable performance metrics;
7. test requirements in MATLAB, Python, and C where appropriate;
8. separate design code from deployment-oriented runtime code;
9. document what the model can and cannot prove.

## Projects

| Project | Main methods | Published result |
|---|---|---|
| [Nonlinear control loops](projects/nonlinear-control-loops/) | Jacobian linearisation, equilibrium stability, controllability, local state feedback | Linearisation error increases away from the operating point |
| [Elastically mounted rotary arm](projects/rotary-arm/) | Fifth-order trajectory, feedback, feedforward, 2-DOF control, load disturbance | 2-DOF tracking RMSE is about 70% lower |
| [Quarter-car active suspension](projects/active-suspension/) | State-space modelling, LQR, road disturbance, actuator saturation | RMS body acceleration is reduced by about 35% |
| [Magnetic levitation](projects/magnetic-levitation/) | Nonlinear plant, pole placement, Luenberger observer, sensor noise, convergence study, portable C runtime | Position-estimation RMSE is below 0.001 mm |
| [Two-tank process](projects/two-tank-system/) | Nonlinear hydraulics, PI control, saturation, anti-windup | Recovery improves from about 429 s to 312 s |

Detailed values are generated from executable reference models and committed in [docs/results-summary.md](docs/results-summary.md).

## Preview results

| Active suspension | Magnetic levitation | Two-tank anti-windup |
|---|---|---|
| ![Active suspension](projects/active-suspension/figures/active_suspension_results.svg) | ![Magnetic levitation](projects/magnetic-levitation/figures/magnetic_levitation_observer_results.svg) | ![Two-tank](projects/two-tank-system/figures/two_tank_results.svg) |

## Repository architecture

```text
projects/                  MATLAB experiments and project-specific functions
matlab/+control_lab/       Shared MATLAB RK4, saturation, tracking and recovery utilities
matlab/tests/              Direct matlab.unittest verification
validation/                Independent Python reference models and generated metrics
tests/                     Python numerical and physical-behaviour tests
c/                         Portable C99 controller and observer runtime
c/tests/                   CTest-based runtime verification
docs/                      Architecture, results, traceability and reconstruction notes
.github/workflows/         Python, MATLAB, and C CI workflows
```

See [docs/architecture.md](docs/architecture.md), [docs/verification-matrix.md](docs/verification-matrix.md), and [docs/source-reconstruction-map.md](docs/source-reconstruction-map.md).

## Technical coverage

- Nonlinear differential-equation modelling
- Operating-point and Jacobian linearisation
- State-space modelling, controllability and observability
- Pole and eigenvalue analysis
- Luenberger state observers and output-feedback control
- Deterministic sensor-noise injection
- P, PI, state-feedback, pole-placement and LQR concepts
- Smooth reference trajectories and two-degree-of-freedom control
- Disturbance rejection
- Actuator saturation and back-calculation anti-windup
- Fourth-order Runge-Kutta simulation and step-size convergence
- Portable C99 control runtime with fixed-size memory
- CMake, CTest, GCC, Clang, and strict compiler warnings
- Reproducible performance metrics and three-language verification

## Quick start

### MATLAB

From the repository root:

```matlab
run_all
```

Run the modular observer experiment:

```matlab
run('projects/magnetic-levitation/magnetic_levitation_demo.m')
```

Run MATLAB tests locally:

```matlab
results = runtests('matlab/tests', 'IncludeSubfolders', true);
assertSuccess(results);
```

Recommended environment:

- MATLAB R2021b or later for local runs
- CI uses MATLAB R2024b
- Control System Toolbox is optional; verified fallback gains are included
- No `.slx` files are required for the clean-room demonstrations

### Portable C runtime

```bash
cmake -S c -B build/c -DCMAKE_BUILD_TYPE=Release
cmake --build build/c --parallel
ctest --test-dir build/c --output-on-failure
./build/c/maglev_observer_demo
```

The C implementation uses no dynamic allocation and is compiled with strict warnings under both GCC and Clang.

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

## Why MATLAB, Python, and C are all present

MATLAB is the primary modelling and controller-design environment. It is not the C language, even though MATLAB's runtime and many numerical libraries are implemented in C/C++ and Fortran and MATLAB can generate or call C/C++.

Python independently mirrors the equations for regression checks, generated evidence, and automation. This independence is useful because it can reveal errors that a direct line-by-line translation might repeat.

C is used for deployment-oriented control logic where deterministic memory use, strict compilation, cyclic execution, and later embedded integration matter.

The languages therefore have different roles rather than competing roles.

## Validation strategy

- MATLAB R2024b executes direct `matlab.unittest` tests.
- Python 3.10–3.12 executes independent numerical and documentation checks.
- GCC and Clang compile and test the portable C runtime with warnings treated as errors.
- Noise-free scenarios use tolerance-based cross-language requirements.
- Language-specific random-number generators are not expected to produce identical noise samples.

## Limitations

These are educational models and embedded-oriented reconstructions, not production controllers. They do not establish:

- hardware-in-the-loop performance;
- real-time scheduling guarantees;
- certified sensor or actuator reliability;
- fixed-point numerical behaviour;
- robust stability across untested parameter ranges;
- MISRA-C compliance;
- functional-safety compliance;
- equivalence to the original team submission.

Each project README states its specific assumptions and missing validation.

## Academic attribution

The historical laboratory exercises were completed in a four-person team. The repository contains new clean-room portfolio reconstructions. See [NOTICE.md](NOTICE.md) and [docs/team-contributions-template.md](docs/team-contributions-template.md).
