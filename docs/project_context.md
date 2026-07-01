# Project Context

## Identity

- Project: Automatic Control Laboratory Projects
- Owner: Sadik Enes Erisen
- Maturity: Portfolio MVP and reproducible engineering reference
- Languages and tools: MATLAB R2024b, Python 3.10–3.12, portable C99, CMake, CTest
- Repository visibility: Public

## Purpose

Provide reproducible control-system experiments that connect nonlinear modelling, linearisation, controller and observer design, numerical validation, and deployment-oriented C implementations.

The repository should demonstrate engineering reasoning and executable evidence rather than a collection of disconnected scripts.

## Current validated scope

- nonlinear systems and operating-point linearisation;
- controllability and observability;
- state feedback, pole placement, LQR, and Luenberger observers;
- actuator saturation and back-calculation anti-windup;
- deterministic sensor-noise and disturbance scenarios;
- RK4 and convergence studies;
- MATLAB experiments and tests;
- independent Python numerical checks and report-freshness validation;
- portable C99 observer and PI-control runtimes;
- GCC and Clang builds with strict warnings;
- requirements-to-test traceability.

## Current projects

- Nonlinear control loops
- Elastically mounted rotary arm
- Quarter-car active suspension
- Magnetic levitation
- Two-tank process

## Core invariants

1. Equations, units, state ordering, signs, initial conditions, and parameter values remain explicit.
2. Published numerical claims are generated from executable models and reproducible commands.
3. Independent validation must not simply copy the implementation under test.
4. MATLAB, Python, and C representations of the same model remain semantically aligned.
5. Simulation evidence is not hardware or safety evidence.
6. Attribution and academic-team provenance remain accurate.

## Out of scope unless separately implemented and verified

- hardware-in-the-loop or physical-plant validation;
- real-time scheduling guarantees;
- fixed-point numerical validation;
- MISRA-C compliance;
- robust stability beyond tested ranges;
- functional-safety certification;
- claims that one controller is universally optimal.

## Interfaces requiring compatibility review

- MATLAB function signatures and project entry points;
- Python validation modules and report schemas;
- C headers, configuration structures, and demo output;
- CMake targets and CTest names;
- generated figure and report paths;
- metric definitions and verification-matrix identifiers.

## Definition of done

- [ ] Acceptance criteria are explicit.
- [ ] Equations and assumptions are documented.
- [ ] Affected MATLAB, Python, and C layers are tested.
- [ ] Important numerical results have analytical, independent, or cross-language evidence.
- [ ] Generated results and figures are current.
- [ ] Verification traceability is updated.
- [ ] Limitations and attribution remain accurate.
- [ ] Enes can explain the model, controller, numerical method, tests, and deployment boundary.
