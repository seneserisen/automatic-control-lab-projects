# Reproducible results summary

Generated from the licence-free Python reference models in `validation/`.
These numbers are educational simulation results, not hardware measurements.

## Nonlinear control loops

- Stable-equilibrium maximum pole real part: `-0.5000 1/s`
- Unstable-equilibrium maximum pole real part: `1.2361 1/s`
- Closed-loop maximum pole real part: `-2.0000 1/s`
- Controllability rank: `2`
- Near-operating-point linearisation RMSE: `0.000020`
- Farther-operating-point linearisation RMSE: `0.003307`

The larger error for the farther initial condition demonstrates that linearisation accuracy is local.

## Rotary arm

- Feedback-only tracking RMSE: `0.012797 rad`
- Two-degree-of-freedom tracking RMSE: `0.003787 rad`
- Tracking-RMSE improvement: `70.40%`
- Peak two-degree-of-freedom control magnitude: `4.401`

The two-degree-of-freedom design combines model-based feedforward with feedback disturbance rejection.

## Active suspension

- Passive body-acceleration RMS: `2.3470 m/s²`
- Active body-acceleration RMS: `1.5236 m/s²`
- RMS acceleration reduction: `35.08%`
- Peak active force: `5.830 N`

The result quantifies ride-comfort improvement while exposing the required actuator effort.

## Magnetic levitation: local-model validation

- Equilibrium current: `2.0011 A`
- Equilibrium voltage: `22.0124 V`
- Open-loop maximum pole real part: `37.4357 1/s`
- Closed-loop maximum pole real part: `-20.0164 1/s`
- Linear/nonlinear position RMSE: `0.000909 mm`
- Maximum applied voltage: `22.7985 V`

The nonlinear comparison verifies that the local controller remains accurate for the selected 0.5 mm reference step.

## Magnetic levitation: output-feedback observer

- Measured states: gap and coil current
- Estimated state: ball velocity
- Observability rank: `3`
- Observer maximum pole real part: `-80.0000 1/s`
- Full-state tracking RMSE: `0.191583 mm`
- Observer-based tracking RMSE: `0.189870 mm`
- Position-estimation RMSE: `0.000649 mm`
- Velocity-estimation RMSE: `0.00003398 m/s`
- Current-estimation RMSE: `0.00010277 A`
- Maximum observer-control voltage: `22.8253 V`
- 0.1 ms versus 0.05 ms final-position difference: `0.000004 mm`
- 0.1 ms versus 0.05 ms maximum-voltage difference: `0.000057 V`

The observer uses deterministic measurement noise with seed 42. The convergence study disables noise so integration-step effects are not confused with different random samples.

## Two-tank process

- Initial equilibrium levels: `h1=18.1000 cm`, `h2=10.0000 cm`
- Pump-limited maximum steady-state tank-2 level: `31.8552 cm`
- Deliberately unreachable reference: `35.0000 cm`
- Recovery time without anti-windup: `428.65 s`
- Recovery time with back-calculation: `311.65 s`

The scenario begins at equilibrium and uses a reference above the pump's steady-state capability, so the windup comparison has a clear physical basis.
