# Magnetic Levitation Control

Magnetic levitation is an unstable nonlinear control problem. This reconstruction calculates the equilibrium current, builds a local deviation model and demonstrates stabilising state feedback.

## Verified laboratory parameters

- Ball mass: `0.068 kg`
- Equilibrium air gap: `0.014 m`
- Magnetic coefficient: `6.53e-5`
- Coil inductance: `0.4125 H`
- Total resistance: `11 ohm`

The calculated equilibrium current is approximately `2.001 A`.

## What the script does

- calculates the equilibrium current;
- constructs the unstable linearised magnetic/electrical model;
- prints open- and closed-loop poles;
- applies precomputed pole-placement feedback;
- tracks a small air-gap reference;
- plots the magnetic frequency response and Nyquist locus.

## Run

```matlab
magnetic_levitation_demo
```

## Engineering interpretation

The controller is designed for small deviations around the operating point. Real hardware would require sensor filtering, voltage and current limits, derivative estimation, robust stability analysis, and safety shutdown logic.

## Preview

![Magnetic Levitation results](figures/magnetic_levitation_results.svg)
