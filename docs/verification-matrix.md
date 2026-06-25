# Requirements-to-test traceability

This matrix connects engineering claims to automated evidence.

| ID | Requirement | Evidence | MATLAB test | Python test | C test |
|---|---|---|---|---|---|
| CTRL-001 | Shared RK4 integration must reproduce first-order exponential decay within tolerance. | Numerical utility output | `TestControlLabUtilities/rk4MatchesExponentialDecay` | `test_rk4_matches_exponential_decay` | Magnetic-levitation convergence test exercises the C RK4 implementation |
| CTRL-002 | Saturation must enforce inclusive lower and upper bounds. | Utility output | `TestControlLabUtilities/saturationClampsValues` | Existing actuator-limit tests | `test_noise_free_tracking` |
| CTRL-003 | Tracking metrics must report consistent RMSE, IAE, ISE, steady-state error, and control effort. | Metric structure | `TestControlLabUtilities/trackingMetricsAreConsistent` | Generated-results freshness check | Online C RMSE metrics checked by `test_reference_metrics_are_reasonable` |
| MAG-001 | The local magnetic-levitation model must be observable from gap and coil-current measurements. | Observability rank = 3 | `TestMaglevObserver/modelIsObservable` | `test_maglev_observer_design_is_stable_and_observable` | Uses the validated precomputed observer gain |
| MAG-002 | Controller and observer error dynamics must be asymptotically stable. | Pole real parts < 0 | `TestMaglevObserver/controllerAndObserverAreStable` | `test_maglev_observer_design_is_stable_and_observable` | Runtime tracking and bounded-state checks |
| MAG-003 | Observer-based control must track the 0.5 mm command on the nonlinear plant with deterministic sensor noise. | Final position and tracking metrics | `TestMaglevObserver/observerTracksWithSensorNoise` | `test_observer_tracks_reference_with_sensor_noise` | `test_reference_metrics_are_reasonable` |
| MAG-004 | Position-estimation RMSE must remain below 0.002 mm. | Generated metrics | `TestMaglevObserver/observerTracksWithSensorNoise` | `test_observer_tracks_reference_with_sensor_noise` | `test_reference_metrics_are_reasonable` |
| MAG-005 | Applied voltage must remain within 0–30 V. | Maximum/minimum voltage | `TestMaglevObserver/observerTracksWithSensorNoise` | `test_observer_tracks_reference_with_sensor_noise` | `test_noise_free_tracking` |
| MAG-006 | The simulation must be reproducible for a fixed measurement-noise seed. | Repeated equal metrics or trajectories | `TestMaglevObserver/fixedSeedIsDeterministic` | `test_observer_simulation_is_deterministic_for_fixed_seed` | `test_fixed_seed_is_deterministic` |
| MAG-007 | The selected 0.1 ms step must agree closely with the 0.05 ms reference. | Final-position and voltage differences | `TestMaglevObserver/fixedStepSolutionConverges` | `test_observer_fixed_step_solution_converges` | `test_fixed_step_convergence` |
| MAG-008 | The portable runtime must avoid dynamic memory and use fixed-size state storage. | Static source inspection and compiler build | Not applicable | Not applicable | C API and source use fixed-size objects |
| MAG-009 | Invalid runtime configurations must be rejected before simulation. | Nonzero return code | MATLAB argument validation | Python invalid-input test | `test_invalid_configuration` |
| DOC-001 | Published metrics must match the current executable reference models. | `docs/results-summary.md` | Not applicable | `python -m validation.report --check` | C demo prints runtime metrics |
| CI-001 | Python validation must pass on supported Python versions. | GitHub Actions matrix | Not applicable | Python 3.10–3.12 workflow | Not applicable |
| CI-002 | MATLAB utility and observer tests must pass in MATLAB. | GitHub Actions JUnit artifact | Official MATLAB workflow | Not applicable | Not applicable |
| CI-003 | The portable runtime must compile under GCC and Clang with strict warnings and pass CTest. | GitHub Actions matrix | Not applicable | Not applicable | `Validate C control runtime` workflow |

## Interpretation

A passing test verifies the documented simulation or runtime requirement under the committed model and configuration. It does not establish hardware safety, real-time schedulability, fixed-point correctness, robust stability outside the tested parameter ranges, MISRA-C compliance, or functional-safety certification.
