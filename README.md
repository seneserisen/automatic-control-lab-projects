<h1 align="center">Automatic Control Laboratory Projects</h1>

<p align="center">
  <b>Reproducible control-system experiments from nonlinear modelling to portable C99 runtimes</b>
</p>

<p align="center">
  <a href="https://github.com/seneserisen/automatic-control-lab-projects/actions/workflows/validate.yml"><img src="https://github.com/seneserisen/automatic-control-lab-projects/actions/workflows/validate.yml/badge.svg" alt="Python validation" /></a>
  <a href="https://github.com/seneserisen/automatic-control-lab-projects/actions/workflows/matlab.yml"><img src="https://github.com/seneserisen/automatic-control-lab-projects/actions/workflows/matlab.yml/badge.svg" alt="MATLAB validation" /></a>
  <a href="https://github.com/seneserisen/automatic-control-lab-projects/actions/workflows/c.yml"><img src="https://github.com/seneserisen/automatic-control-lab-projects/actions/workflows/c.yml/badge.svg" alt="C validation" /></a>
  <img src="https://img.shields.io/badge/MATLAB-R2024b-EF6C00" alt="MATLAB R2024b" />
  <img src="https://img.shields.io/badge/C-C99-A8B9CC" alt="C99" />
  <img src="https://img.shields.io/badge/Python-3.10--3.12-3776AB" alt="Python 3.10 to 3.12" />
</p>

This repository is a control-engineering portfolio covering **nonlinear systems, state-space control, LQR, observers, actuator saturation, anti-windup, numerical validation, and deployment-oriented C implementations**.

It is designed for engineers and students who want more than isolated scripts: each experiment documents its model, assumptions, controller design, measurable results, automated evidence, and limitations.

## Why this repository is useful

- Compare multiple controller architectures under documented conditions.
- Reproduce numerical results from executable MATLAB and Python models.
- Inspect portable C99 implementations with fixed-size state and no dynamic allocation.
- Trace engineering requirements to automated MATLAB, Python, and C tests.
- Study where simulation evidence ends and hardware validation must begin.

## Featured projects

| Project | Main methods | Published result |
|---|---|---|
| [Nonlinear control loops](projects/nonlinear-control-loops/) | Jacobian linearisation, equilibrium stability, controllability, local state feedback | Linearisation error increases away from the operating point |
| [Elastically mounted rotary arm](projects/rotary-arm/) | Fifth-order trajectory, feedback, feedforward, 2-DOF control, load disturbance | 2-DOF tracking RMSE is about 70% lower |
| [Quarter-car active suspension](projects/active-suspension/) | State-space modelling, LQR, road disturbance, actuator saturation | RMS body acceleration is reduced by about 35% |
| [Magnetic levitation](projects/magnetic-levitation/) | Nonlinear plant, pole placement, Luenberger observer, sensor noise, convergence study, portable C runtime | Position-estimation RMSE is below 0.001 mm |
| [Two-tank process](projects/two-tank-system/) | Nonlinear hydraulics, PI control, saturation, back-calculation, portable C runtime | Recovery improves from about 429 s to 312 s |

Detailed numerical results are generated from executable models and published in [the results summary](docs/results-summary.md).

## Results preview

| Active suspension | Magnetic levitation | Two-tank anti-windup |
|---|---|---|
| ![Active suspension](projects/active-suspension/figures/active_suspension_results.svg) | ![Magnetic levitation](projects/magnetic-levitation/figures/magnetic_levitation_observer_results.svg) | ![Two-tank](projects/two-tank-system/figures/two_tank_results.svg) |

## Quick start

### MATLAB experiments

```matlab
run_all
```

Run the MATLAB test suite:

```matlab
results = runtests('matlab/tests', 'IncludeSubfolders', true);
assertSuccess(results);
```

### Portable C runtimes

```bash
cmake -S c -B build/c -DCMAKE_BUILD_TYPE=Release
cmake --build build/c --parallel
ctest --test-dir build/c --output-on-failure
./build/c/maglev_observer_demo
./build/c/two_tank_demo
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
- Portable C99 observer and PI-control runtimes
- Fixed-size state storage without dynamic allocation
- CMake, CTest, GCC, Clang, and strict compiler warnings
- Direct MATLAB and multi-version Python CI

## Verification approach

| Layer | Evidence |
|---|---|
| MATLAB | Direct `matlab.unittest` execution using MATLAB R2024b |
| Python | Numerical regression, behaviour, and report-freshness checks on Python 3.10–3.12 |
| C | GCC and Clang builds with warnings treated as errors |
| Runtime | CTest checks for configuration handling, tracking, saturation, determinism, anti-windup, and convergence |
| Traceability | Requirements mapped to automated evidence in [the verification matrix](docs/verification-matrix.md) |

## Repository structure

```text
projects/                  MATLAB experiments and project-specific functions
matlab/+control_lab/       Shared MATLAB numerical and metrics utilities
matlab/tests/              Direct matlab.unittest verification
validation/                Independent numerical reference models and reports
tests/                     Python numerical and behaviour tests
c/                         Portable C99 control runtimes
c/tests/                   CTest-based runtime verification
docs/                      Architecture, results, and requirements traceability
.github/workflows/         MATLAB, Python, and C CI workflows
```

See [the architecture document](docs/architecture.md) for the system structure and extension points.

## Contributing

Focused contributions that improve numerical validation, portability, documentation, or test coverage are welcome. Read [CONTRIBUTING.md](CONTRIBUTING.md) before opening a pull request.

Useful starting points include:

- independent numerical checks;
- additional deterministic disturbance scenarios;
- compiler and platform portability improvements;
- documentation corrections;
- requirements-to-test traceability improvements.

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
