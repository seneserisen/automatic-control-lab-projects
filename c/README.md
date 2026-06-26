# Portable C Control Runtimes

Portable C99 implementations of selected control algorithms from the portfolio.

## Implemented runtimes

### Magnetic-levitation observer

- nonlinear plant;
- state-feedback control;
- Luenberger observer;
- deterministic sensor noise;
- 0–30 V saturation;
- online tracking and estimation metrics.

### Two-tank PI controller

- nonlinear Torricelli-flow plant;
- PI control;
- pump saturation;
- optional back-calculation anti-windup;
- disturbance scenario;
- online recovery, saturation, and integrator metrics.

## Common properties

- C99;
- no dynamic allocation;
- fixed-size state storage;
- fixed-step RK4 integration;
- explicit configuration validation;
- CMake and CTest;
- GCC and Clang CI with strict warnings.

## Build and test

```bash
cmake -S c -B build/c -DCMAKE_BUILD_TYPE=Release
cmake --build build/c --parallel
ctest --test-dir build/c --output-on-failure
./build/c/maglev_observer_demo
./build/c/two_tank_demo
```

Compiler checks:

```text
-Wall -Wextra -Wpedantic -Werror
```

## Reference results

### Magnetic levitation

| Metric | Result |
|---|---:|
| Final position | 0.501224 mm |
| Observer tracking RMSE | 0.190917 mm |
| Position-estimation RMSE | 0.000594 mm |
| Maximum voltage | 22.8311 V |

### Two-tank anti-windup

| Metric | Standard PI | Back-calculation |
|---|---:|---:|
| Recovery time | 428.65 s | 311.65 s |
| Peak integral state | 1853.18 | 539.36 |
| Final tank-2 level | 10.2474 cm | 10.1593 cm |

## Tests

CTest verifies:

- invalid configuration rejection;
- actuator and pump limits;
- deterministic execution;
- reference tracking;
- observer-estimation quality;
- hydraulic equilibrium and pump reachability;
- anti-windup recovery improvement;
- fixed-step convergence.

## Limitations

The runtimes do not yet include:

- a real-time scheduler;
- ADC, PWM, or communication drivers;
- fixed-point arithmetic;
- sensor calibration or diagnostic states;
- watchdog and emergency shutdown logic;
- MISRA-C analysis;
- hardware-in-the-loop validation;
- generated-code equivalence evidence.

These are embedded-oriented reference implementations, not certified controller firmware.
