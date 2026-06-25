# Portable C Control Runtime

This directory contains deployment-oriented C implementations derived from the validated MATLAB control designs.

## Why C is included

MATLAB is the modelling, analysis, and controller-design environment. Python is an independent numerical reference used for reproducible regression tests. C is the portable implementation layer closest to embedded deployment.

The three layers have different responsibilities:

| Layer | Primary purpose |
|---|---|
| MATLAB | Model derivation, controller and observer design, engineering plots, direct numerical tests |
| Python | Independent reference implementation, generated evidence, multi-version regression checks |
| C | Deterministic runtime structure, fixed memory use, strict compiler checks, embedded-oriented implementation |

Python is therefore not a substitute for C. It remains useful because an independent implementation can detect errors that are repeated when code is translated directly from MATLAB.

## Current implementation

The magnetic-levitation observer runtime includes:

- C99 source code;
- no dynamic allocation;
- fixed-size state and output vectors;
- nonlinear plant equations;
- precomputed state-feedback and observer gains;
- fixed-step RK4 integration;
- 0–30 V actuator saturation;
- deterministic xorshift32 and Box–Muller sensor noise;
- online tracking and estimation metrics;
- configuration validation;
- CMake and CTest support.

## Build and test

```bash
cmake -S c -B build/c -DCMAKE_BUILD_TYPE=Release
cmake --build build/c --parallel
ctest --test-dir build/c --output-on-failure
./build/c/maglev_observer_demo
```

The CI workflow compiles the same code with both GCC and Clang using:

```text
-Wall -Wextra -Wpedantic -Werror
```

## Reference results

The C runtime uses the same physical parameters, controller gains, observer gains, sample time, reference, noise magnitudes, and voltage limits as the MATLAB experiment. The random-number generator is intentionally C-specific, so individual noise samples are not expected to match MATLAB or NumPy exactly.

Representative C results with seed 42:

- final position: approximately 0.501224 mm;
- observer tracking RMSE: approximately 0.190917 mm;
- position-estimation RMSE: approximately 0.000594 mm;
- maximum voltage: approximately 22.8311 V.

The acceptance tests compare physical and numerical requirements rather than demanding bitwise equality across unrelated random-number generators.

## Not yet production-ready

This implementation does not yet include:

- a real-time scheduler;
- ADC or PWM drivers;
- fixed-point arithmetic;
- sensor calibration or diagnostic states;
- watchdog and emergency shutdown logic;
- MISRA-C analysis;
- hardware-in-the-loop validation;
- generated code equivalence evidence.

It is an embedded-oriented portfolio implementation, not certified controller firmware.
