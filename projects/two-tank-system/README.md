# Two-Tank Process Control

This experiment creates a physically grounded actuator-saturation scenario and compares a standard PI controller with back-calculation anti-windup.

## Model

\[
q_{12}=a_{12}\sqrt{2g\max(h_1-h_2,0)}
\]

\[
q_{out}=a_2\sqrt{2g\max(h_2,0)}
\]

The simulation starts at the hydraulic equilibrium corresponding to `h2=10 cm`, rather than filling from empty tanks.

## Physical reachability check

With the pump limited to `45 cm³/s`, the maximum steady-state tank-2 level is approximately `31.855 cm`. The experiment commands `35 cm`, making the saturation interval demonstrably unreachable.

## Experiment workflow

1. calculate the initial equilibrium levels and PI integral state;
2. command an unreachable reference to force sustained saturation;
3. return the reference to 10 cm;
4. compare recovery with and without back-calculation;
5. apply a later inlet disturbance after the recovery comparison.

## Reproducible results

- Recovery without anti-windup: approximately `428.65 s`
- Recovery with back-calculation: approximately `311.65 s`

Back-calculation reduces the recovery time by limiting excessive integral accumulation during saturation.

## Run

```matlab
two_tank_demo
```

## Assumptions and limitations

- flows are quasi-steady and follow Torricelli’s law;
- pump dynamics and measurement noise are omitted;
- negative inter-tank flow is not allowed;
- controller tuning is illustrative rather than plant-optimised;
- the disturbance is modelled as an inlet-flow reduction.

## Preview

![Two Tank System results](figures/two_tank_results.svg)
