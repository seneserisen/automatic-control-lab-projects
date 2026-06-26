# Two-Tank Process Control

This project creates a physically grounded actuator-saturation scenario and compares a standard PI controller with back-calculation anti-windup. The controller is implemented in MATLAB and as a portable C99 runtime.

## Model

\[
q_{12}=a_{12}\sqrt{2g\max(h_1-h_2,0)}
\]

\[
q_{out}=a_2\sqrt{2g\max(h_2,0)}
\]

The simulation starts at the hydraulic equilibrium corresponding to `h2=10 cm`, rather than filling from empty tanks.

## Controller

Standard PI law:

\[
u^*=K_p e + K_i x_I
\]

Pump saturation:

\[
u=\operatorname{sat}(u^*,0,45)
\]

Back-calculation anti-windup:

\[
\dot{x}_I=e+K_{aw}(u-u^*)
\]

| Parameter | Value |
|---|---:|
| Proportional gain | 5.0 |
| Integral gain | 0.05 |
| Anti-windup gain | 0.30 |
| Pump range | 0–45 cm³/s |
| Sample time | 0.05 s |

## Physical reachability

With the pump limited to `45 cm³/s`, the maximum steady-state tank-2 level is approximately `31.855 cm`. The experiment commands `35 cm`, making the saturation interval physically unreachable.

## Experiment sequence

1. Initialise both tanks and the PI integrator at hydraulic equilibrium.
2. Command 35 cm from 100 s to 160 s to force sustained saturation.
3. Return the reference to 10 cm.
4. Measure sustained recovery with and without back-calculation.
5. Apply a later inlet disturbance from 520 s to 580 s.

## Results

| Metric | Standard PI | Back-calculation |
|---|---:|---:|
| Recovery time | 428.65 s | 311.65 s |
| Peak integral state | 1853.18 | 539.36 |
| Saturation duration | 84.05 s | 91.45 s |
| Final tank-2 level | 10.2474 cm | 10.1593 cm |

Back-calculation improves sustained recovery by approximately `117 s` and reduces peak integrator growth by approximately `71%`.

The anti-windup controller may remain saturated for slightly longer because it deliberately prevents the integral state from accumulating the excessive control demand that would later delay recovery. Saturation duration alone is therefore not the performance objective.

## Run MATLAB

```matlab
two_tank_demo
```

## Build and run C

From the repository root:

```bash
cmake -S c -B build/c -DCMAKE_BUILD_TYPE=Release
cmake --build build/c --parallel
ctest --test-dir build/c --output-on-failure
./build/c/two_tank_demo
```

The C99 runtime uses fixed-size state storage, no dynamic allocation, RK4 integration, explicit pump saturation, and online recovery metrics.

## Automated verification

CTest verifies:

- invalid configuration rejection;
- hydraulic equilibrium and pump reachability;
- documented recovery times;
- anti-windup reduction of integrator growth;
- pump-command limits;
- return to the nominal operating region;
- fixed-step convergence.

The runtime is compiled with GCC and Clang using strict warnings.

## Assumptions and limitations

- flows are quasi-steady and follow Torricelli’s law;
- pump dynamics and measurement noise are omitted;
- negative inter-tank flow is not allowed;
- controller tuning is illustrative rather than plant-optimised;
- the disturbance is modelled as an inlet-flow reduction;
- real sensor calibration, pump dead time, and embedded scheduling are not represented.

## Preview

![Two Tank System results](figures/two_tank_results.svg)
