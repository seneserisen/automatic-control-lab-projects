# Manual MATLAB validation checklist

The Python reference simulations pass automated tests, but the MATLAB scripts should be executed before the repository is described as MATLAB-validated.

## Environment

- [ ] Record MATLAB release and operating system.
- [ ] Clone or download the repository into a path without unusual characters.
- [ ] Open MATLAB at the repository root.

## Full run

```matlab
run_all
```

Verify:

- [ ] All five scripts finish without errors.
- [ ] Each script writes one PNG into its project `figures/` directory.
- [ ] Printed poles/eigenvalues match the README explanations.
- [ ] No control signal exceeds its documented saturation limit.

## Project checks

### Nonlinear control loops

- [ ] The `[0,0]` equilibrium is locally stable.
- [ ] The `[1,1]` equilibrium has one unstable pole.
- [ ] Feedback poles are `-2` and `-3`.
- [ ] Nonlinear and linearised responses agree near the operating point.

### Rotary arm

- [ ] Final reference is `1 rad`.
- [ ] Tracking error converges close to zero.
- [ ] Feedforward and total input remain below the actuator limit.

### Active suspension

- [ ] Active body acceleration peak is lower than passive peak.
- [ ] The closed-loop poles have negative real parts.
- [ ] The road bump is 20 mm and lasts 0.2 s.

### Magnetic levitation

- [ ] Equilibrium current is approximately `2.001 A`.
- [ ] Open-loop model contains an unstable pole.
- [ ] Closed-loop poles are approximately `-20`, `-30`, and `-40`.
- [ ] The 1 mm reference is tracked without voltage saturation.

### Two-tank system

- [ ] Both cases saturate during the unreachable reference.
- [ ] Back-calculation limits the integral state.
- [ ] Anti-windup returns to the 10 cm reference faster.

## Publication rule

Only mark the repository as MATLAB-validated after completing this checklist and committing any required corrections.
