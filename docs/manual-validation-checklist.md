# Manual MATLAB validation checklist

The repository now runs direct `matlab.unittest` checks in GitHub Actions and retains an independent Python validation layer. This checklist remains the final human review for complete demonstrations, figures and engineering interpretation.

## Automated MATLAB gate

Confirm on the latest commit:

- [ ] `Validate MATLAB experiments` passed.
- [ ] MATLAB utility tests passed.
- [ ] Magnetic-levitation observer tests passed.
- [ ] The JUnit artifact was generated.
- [ ] No strict-mode warnings were reported.

## Environment

- [ ] Record MATLAB release and operating system.
- [ ] Clone the repository into a path without unusual characters.
- [ ] Open MATLAB at the repository root.
- [ ] Confirm `matlab/+control_lab/` and `matlab/tests/` are present.

## Local tests

```matlab
results = runtests('matlab/tests', 'IncludeSubfolders', true);
assertSuccess(results);
```

Verify:

- [ ] All local MATLAB unit tests pass.
- [ ] Local test results agree with CI.

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
- [ ] Controller poles are near `-20`, `-30`, and `-40`.
- [ ] Observer poles are near `-80`, `-90`, and `-100`.
- [ ] Observability rank is 3 using gap and current outputs.
- [ ] Observer-based tracking reaches the 0.5 mm command.
- [ ] Estimated velocity follows the true simulated velocity.
- [ ] Position-estimation RMSE remains below 0.002 mm.
- [ ] Total voltage remains between 0 V and 30 V.
- [ ] Repeating the run with seed 42 reproduces the same trajectory.
- [ ] The 0.1 ms and 0.05 ms convergence results closely agree.

### Two-tank system

- [ ] Initial levels are approximately `h1=18.1 cm`, `h2=10 cm`.
- [ ] The 35 cm reference exceeds the calculated pump capability.
- [ ] Both controllers saturate during the high reference.
- [ ] Back-calculation limits integral windup.
- [ ] Anti-windup recovery is faster than the standard PI case.
- [ ] The later inlet disturbance occurs after the recovery comparison.

## Publication rule

After CI and this checklist pass, record the MATLAB release, operating system and validation date in a release note. Continue to describe all results as simulation results rather than hardware measurements.
