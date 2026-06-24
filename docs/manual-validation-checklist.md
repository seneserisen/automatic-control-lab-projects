# Manual MATLAB validation checklist

The Python reference layer passes automated linting and numerical tests. The MATLAB experiments must still be executed before the repository is described as MATLAB-validated.

## Environment

- [ ] Record MATLAB release and operating system.
- [ ] Clone the repository into a path without unusual characters.
- [ ] Open MATLAB at the repository root.
- [ ] Confirm `matlab/+control_lab/` is present.

## Full run

```matlab
run_all
```

Verify:

- [ ] All five scripts finish without errors.
- [ ] Each script writes a PNG into its project `figures/` directory.
- [ ] No `NaN` or `Inf` appears in printed values or plots.
- [ ] Reported metrics are close to `docs/results-summary.md`.

## Project checks

### Nonlinear control loops

- [ ] The `[0,0]` equilibrium is locally stable.
- [ ] The `[1,1]` equilibrium has one unstable pole.
- [ ] The controllability rank is 2.
- [ ] Closed-loop poles are `-2` and `-3`.
- [ ] Farther initial conditions create larger linearisation error.

### Rotary arm

- [ ] Final reference is `1 rad`.
- [ ] 2-DOF RMSE is lower than feedback-only RMSE.
- [ ] The load disturbance occurs from 3.6 s to 3.9 s.
- [ ] Control magnitude remains below 25.

### Active suspension

- [ ] Active body-acceleration RMS is lower than passive RMS.
- [ ] Closed-loop poles have negative real parts.
- [ ] The road bump is 20 mm and lasts 0.2 s.
- [ ] Active force remains below 50 N.
- [ ] Suspension travel and tire deflection remain finite.

### Magnetic levitation

- [ ] Equilibrium current is approximately `2.001 A`.
- [ ] Equilibrium voltage is approximately `22.012 V`.
- [ ] The open-loop model contains an unstable pole.
- [ ] Closed-loop poles are near `-20`, `-30`, and `-40`.
- [ ] Linear and nonlinear 0.5 mm responses closely agree.
- [ ] Total voltage remains between 0 V and 30 V.

### Two-tank system

- [ ] Initial levels are approximately `h1=18.1 cm`, `h2=10 cm`.
- [ ] The 35 cm reference exceeds the calculated pump capability.
- [ ] Both controllers saturate during the high reference.
- [ ] Back-calculation limits integral windup.
- [ ] Anti-windup recovery is faster than the standard PI case.
- [ ] The later inlet disturbance occurs after the recovery comparison.

## Publication rule

Only mark the repository as MATLAB-validated after completing this checklist and committing any required corrections. Record the MATLAB release and validation date in the README or a release note.
