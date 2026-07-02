# Project Status

- Last updated: 2 July 2026
- Maturity: Portfolio MVP and reproducible control-engineering reference
- Default branch: `main`

## Implemented and documented

- nonlinear control-loop experiments;
- elastically mounted rotary-arm tracking comparisons;
- quarter-car active-suspension LQR analysis;
- magnetic-levitation observer design and portable C runtime;
- two-tank PI control, saturation, anti-windup, and portable C runtime;
- MATLAB unit tests;
- independent Python numerical and report validation;
- portable C99 builds and CTest validation;
- Python 3.10–3.12, MATLAB, GCC, and Clang CI;
- results summary and requirements-to-test verification matrix.

## Current validation boundary

The repository contains simulation and software-validation evidence. It does not establish:

- physical-plant or hardware-in-the-loop performance;
- real-time scheduling guarantees;
- fixed-point behavior;
- MISRA-C compliance;
- robust stability outside tested parameter ranges;
- functional-safety certification.

## Main maintenance risks

- numerical drift between MATLAB, Python, and C representations;
- stale generated figures or reports after model changes;
- tolerance changes that hide real regressions;
- state-order, unit, sign, or sample-time inconsistencies;
- loss of attribution or confusion between team coursework and independent portfolio restructuring;
- claims that exceed the simulation boundary.

## Highest-value next improvements

1. Add explicit parameter and state-schema tables for every project.
2. Strengthen cross-language reference tests for the C runtimes.
3. Add property or sweep tests for integration-step and parameter sensitivity.
4. Add static analysis appropriate to portable C without claiming MISRA compliance.
5. Review generated results for reproducibility on a clean environment.

## Verification status

| Layer | Current evidence | Remaining boundary |
|---|---|---|
| MATLAB | Direct tests and executable experiments | Toolboxes and version must remain documented |
| Python | Multi-version numerical and report validation | Independent logic must remain independent |
| C | GCC/Clang, CMake, CTest, demo runtimes | No real-time or fixed-point guarantee |
| Hardware | None | Separate future milestone |

## Active governance work

The `chore/ai-project-governance` branch adds project-specific agent instructions and documentation controls without changing numerical implementation or published results.
