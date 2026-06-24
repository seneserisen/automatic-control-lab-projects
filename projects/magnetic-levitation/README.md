# Magnetic Levitation Control

Magnetic levitation is nonlinear and open-loop unstable. This experiment designs a local state-feedback controller and verifies it against the nonlinear plant rather than evaluating only the linearised equations.

## Parameters

| Parameter | Value |
|---|---:|
| Ball mass | 0.068 kg |
| Equilibrium gap | 0.014 m |
| Magnetic coefficient | 6.53×10⁻⁵ |
| Coil inductance | 0.4125 H |
| Total resistance | 11 Ω |
| Voltage range | 0–30 V |

The equilibrium current is approximately `2.0011 A`, requiring about `22.0124 V` at steady state.

## Experiment workflow

1. calculate the equilibrium current and voltage;
2. derive the local three-state model;
3. inspect the unstable open-loop poles;
4. apply pole-placement feedback with desired poles near `-20`, `-30` and `-40`;
5. simulate the linear and nonlinear plants independently with RK4;
6. compare their 0.5 mm reference responses and voltage limits.

## Reproducible results

- Open-loop maximum pole real part: approximately `37.44 1/s`
- Closed-loop maximum pole real part: approximately `-20.02 1/s`
- Linear/nonlinear position RMSE: approximately `0.000909 mm`
- Maximum applied voltage: approximately `22.7985 V`

## Run

```matlab
magnetic_levitation_demo
```

## Assumptions and limitations

- full gap, velocity and current states are assumed available;
- the controller is valid only near the 14 mm equilibrium;
- magnetic saturation, eddy currents and sensor dynamics are omitted;
- contact, drop and emergency shutdown behaviour are not simulated;
- robustness to parameter error is not yet quantified.

## Preview

![Magnetic Levitation results](figures/magnetic_levitation_results.svg)
