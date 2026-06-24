# Quarter-Car Active Suspension

This project models a laboratory-scale quarter car and compares passive behaviour with active state feedback.

## States

\[
x=[z_s,\dot z_s,z_u,\dot z_u]^T
\]

where `z_s` is sprung-mass displacement and `z_u` is unsprung-mass displacement.

## What the script does

- builds the coupled four-state model;
- injects a smooth road bump and its velocity;
- simulates the passive suspension;
- applies a precomputed LQR feedback gain with actuator saturation;
- compares body displacement, body acceleration and suspension travel.

## Run

```matlab
active_suspension_demo
```

## Engineering interpretation

The controller reduces body acceleration and suspension travel in this simplified scenario. A real design would also constrain tire load variation, actuator bandwidth, energy consumption, sensor noise and model uncertainty.

## Preview

![Active Suspension results](figures/active_suspension_results.svg)
