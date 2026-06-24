# Two-Tank Process Control

This project models two hydraulically coupled tanks and compares a standard PI controller with back-calculation anti-windup.

## Model

The inter-tank and outlet flows use Torricelli’s law:

\[
q_{12}=a_{12}\sqrt{2g\max(h_1-h_2,0)}
\]

\[
q_{out}=a_2\sqrt{2g\max(h_2,0)}.
\]

## Scenario

The level reference is temporarily raised beyond what the saturated pump can achieve. When the reference returns to a reachable value, the controller without anti-windup recovers slowly because its integral state has accumulated excessive error.

## Run

```matlab
two_tank_demo
```

## Engineering interpretation

Back-calculation feeds the difference between saturated and unsaturated control back into the integrator. It reduces windup and improves recovery after saturation.

## Preview

![Two Tank System results](figures/two_tank_results.svg)
