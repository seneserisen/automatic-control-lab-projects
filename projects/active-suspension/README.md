# Quarter-Car Active Suspension

This experiment evaluates whether LQR state feedback improves ride comfort for a laboratory-scale quarter-car model while respecting an actuator-force limit.

## States

\[
x=[z_s,\dot z_s,z_u,\dot z_u]^T
\]

where `z_s` is sprung-mass displacement and `z_u` is unsprung-mass displacement.

## Parameters

| Parameter | Value |
|---|---:|
| Sprung mass | 2.45 kg |
| Unsprung mass | 1.40 kg |
| Suspension stiffness | 900 N/m |
| Tire stiffness | 2500 N/m |
| Suspension damping | 12 N·s/m |
| Tire damping | 10 N·s/m |
| Actuator-force limit | ±50 N |

## Experiment workflow

1. build the four-state model;
2. generate a smooth 20 mm road bump;
3. compute the LQR gain when the toolbox is available, otherwise use the documented fallback gain;
4. simulate passive and active systems with RK4;
5. compare body acceleration, suspension travel, tire deflection and actuator force.

## Reproducible results

- Passive body-acceleration RMS: approximately `2.3470 m/s²`
- Active body-acceleration RMS: approximately `1.5236 m/s²`
- RMS reduction: approximately `35.1%`
- Peak active force: approximately `5.830 N`

## Run

```matlab
active_suspension_demo
```

## Assumptions and limitations

- all four states are measured directly;
- the road profile is deterministic and perfectly known at the tire;
- actuator bandwidth and energy use are not modelled;
- the LQR weights are educational choices, not optimised vehicle requirements;
- tire-load variation and robustness to parameter uncertainty require further analysis.

## Preview

![Active Suspension results](figures/active_suspension_results.svg)
