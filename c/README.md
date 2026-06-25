# Portable C Control Runtime

A portable C99 implementation of the magnetic-levitation observer and controller.

## Features

- no dynamic allocation;
- fixed-size state and output vectors;
- nonlinear magnetic-levitation plant;
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

CI compiles the runtime with GCC and Clang using:

```text
-Wall -Wextra -Wpedantic -Werror
```

## Reference results

| Metric | Result |
|---|---:|
| Final position | 0.501224 mm |
| Observer tracking RMSE | 0.190917 mm |
| Position-estimation RMSE | 0.000594 mm |
| Velocity-estimation RMSE | 0.00003607 m/s |
| Current-estimation RMSE | 0.00009038 A |
| Maximum voltage | 22.8311 V |

## Tests

CTest verifies:

- invalid configuration rejection;
- noise-free reference tracking;
- actuator-voltage limits;
- deterministic seeded execution;
- fixed-step convergence;
- bounded tracking and estimation metrics.

## Limitations

The runtime does not yet include:

- a real-time scheduler;
- ADC or PWM drivers;
- fixed-point arithmetic;
- sensor calibration or diagnostic states;
- watchdog and emergency shutdown logic;
- MISRA-C analysis;
- hardware-in-the-loop validation;
- generated-code equivalence evidence.

This is an embedded-oriented reference implementation, not certified controller firmware.
