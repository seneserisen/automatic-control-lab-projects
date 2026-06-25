# Magnetic Levitation Control

Magnetic levitation is nonlinear and open-loop unstable. This project progresses from local full-state feedback to an **output-feedback observer** that measures air gap and coil current while estimating ball velocity. The validated design is also implemented as a portable C99 runtime.

## Engineering objective

Stabilise the nonlinear plant near a 14 mm operating gap and track a 0.5 mm command without direct velocity measurement, while accounting for sensor noise and voltage limits.

## Parameters

| Parameter | Value |
|---|---:|
| Ball mass | 0.068 kg |
| Equilibrium gap | 0.014 m |
| Magnetic coefficient | 6.53×10⁻⁵ |
| Coil inductance | 0.4125 H |
| Total resistance | 11 Ω |
| Voltage range | 0–30 V |
| Gap-noise standard deviation | 0.005 mm |
| Current-noise standard deviation | 0.002 A |
| Noise seed | 42 |

The equilibrium current is approximately `2.0011 A`, requiring about `22.0124 V` at steady state.

## MATLAB architecture

```text
maglev_configuration.m
        ↓
maglev_linear_model.m
        ↓
design_maglev_observer.m
        ↓
simulate_maglev_observer.m
        ↓
calculate_maglev_observer_metrics.m
        ↓
plot_maglev_observer_results.m
```

`magnetic_levitation_demo.m` coordinates the experiment. Plant dynamics, controller design, observer design, simulation, metrics, and plotting are independently testable functions.

## Controller and observer

Controller poles:

```text
-20, -30, -40
```

Observer poles:

```text
-80, -90, -100
```

Measured outputs:

- air-gap deviation;
- coil-current deviation.

Estimated state:

- ball velocity.

Control law:

\[
u=-K\hat{x}+N_r r
\]

Observer:

\[
\dot{\hat{x}}=A\hat{x}+Bu+L(y-C\hat{x}).
\]

## Results

### MATLAB/Python reference

| Metric | Result |
|---|---:|
| Observability rank | 3 |
| Full-state tracking RMSE | 0.191583 mm |
| Observer-based tracking RMSE | 0.189870 mm |
| Position-estimation RMSE | 0.000649 mm |
| Velocity-estimation RMSE | 0.00003398 m/s |
| Current-estimation RMSE | 0.00010277 A |
| Maximum control voltage | 22.8253 V |

### Portable C runtime

| Metric | Result |
|---|---:|
| Final position | 0.501224 mm |
| Tracking RMSE | 0.190917 mm |
| Position-estimation RMSE | 0.000594 mm |
| Velocity-estimation RMSE | 0.00003607 m/s |
| Current-estimation RMSE | 0.00009038 A |
| Maximum voltage | 22.8311 V |

### Numerical convergence

A no-noise fixed-step study compares 0.8, 0.4, 0.2, 0.1, and 0.05 ms steps.

Between 0.1 ms and 0.05 ms:

- final-position difference: approximately `0.000004 mm`;
- maximum-voltage difference: approximately `0.000057 V`.

## Run MATLAB

```matlab
magnetic_levitation_demo
```

Run only the convergence study:

```matlab
study = maglev_convergence_study();
```

## Build and run C

```bash
cmake -S c -B build/c -DCMAKE_BUILD_TYPE=Release
cmake --build build/c --parallel
ctest --test-dir build/c --output-on-failure
./build/c/maglev_observer_demo
```

## Automated verification

### MATLAB

- observability;
- controller and observer stability;
- noisy output-feedback tracking;
- deterministic seeded noise;
- voltage limits;
- fixed-step convergence;
- shared numerical utilities.

### Python

- observer stability and observability;
- tracking and estimation quality;
- deterministic seeds;
- invalid-input handling;
- fixed-step convergence.

### C

- configuration rejection;
- noise-free tracking;
- voltage saturation;
- deterministic seeded execution;
- fixed-step convergence;
- bounded tracking and estimation metrics.

## Assumptions and limitations

- gap and coil current are measured directly;
- sensor noise is simplified Gaussian white noise;
- the observer uses the local linear model while controlling the nonlinear plant;
- the controller is valid only near the 14 mm equilibrium;
- magnetic saturation, eddy currents, and sensor dynamics are omitted;
- voltage delay, quantisation, scheduling jitter, and emergency shutdown behaviour are not simulated;
- the C implementation is not fixed-point, MISRA-C checked, or connected to real ADC/PWM drivers;
- no hardware or safety-certification claim is made.

## Preview

![Magnetic Levitation observer results](figures/magnetic_levitation_observer_results.svg)
