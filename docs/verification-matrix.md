# Requirements-to-test traceability

This matrix connects engineering claims to automated evidence.

| ID | Requirement | Evidence | MATLAB test | Python test |
|---|---|---|---|---|
| CTRL-001 | Shared RK4 integration must reproduce first-order exponential decay within tolerance. | Numerical utility output | `TestControlLabUtilities/rk4MatchesExponentialDecay` | `test_rk4_matches_exponential_decay` |
| CTRL-002 | Saturation must enforce inclusive lower and upper bounds. | Utility output | `TestControlLabUtilities/saturationClampsValues` | Existing actuator-limit tests |
| CTRL-003 | Tracking metrics must report consistent RMSE, IAE, ISE, steady-state error, and control effort. | Metric structure | `TestControlLabUtilities/trackingMetricsAreConsistent` | Generated-results freshness check |
| MAG-001 | The local magnetic-levitation model must be observable from gap and coil-current measurements. | Observability rank = 3 | `TestMaglevObserver/modelIsObservable` | `test_maglev_observer_design_is_stable_and_observable` |
| MAG-002 | Controller and observer error dynamics must be asymptotically stable. | Pole real parts < 0 | `TestMaglevObserver/controllerAndObserverAreStable` | `test_maglev_observer_design_is_stable_and_observable` |
| MAG-003 | Observer-based control must track the 0.5 mm command on the nonlinear plant with deterministic sensor noise. | Final position and tracking metrics | `TestMaglevObserver/observerTracksWithSensorNoise` | `test_observer_tracks_reference_with_sensor_noise` |
| MAG-004 | Position-estimation RMSE must remain below 0.002 mm. | Generated metrics | `TestMaglevObserver/observerTracksWithSensorNoise` | `test_observer_tracks_reference_with_sensor_noise` |
| MAG-005 | Applied voltage must remain within 0–30 V. | Maximum/minimum voltage | `TestMaglevObserver/observerTracksWithSensorNoise` | `test_observer_tracks_reference_with_sensor_noise` |
| MAG-006 | The simulation must be reproducible for a fixed measurement-noise seed. | Bitwise-equal trajectories | `TestMaglevObserver/fixedSeedIsDeterministic` | `test_observer_simulation_is_deterministic_for_fixed_seed` |
| MAG-007 | The selected 0.1 ms step must agree closely with the 0.05 ms reference. | Final-position and voltage differences | `TestMaglevObserver/fixedStepSolutionConverges` | `test_observer_fixed_step_solution_converges` |
| DOC-001 | Published metrics must match the current executable reference models. | `docs/results-summary.md` | Not applicable | `python -m validation.report --check` |
| CI-001 | Python validation must pass on supported Python versions. | GitHub Actions matrix | Not applicable | Python 3.10–3.12 workflow |
| CI-002 | MATLAB utility and observer tests must pass in a real MATLAB runtime. | GitHub Actions JUnit artifact | Official MATLAB workflow | Not applicable |

## Interpretation

A passing test verifies the documented simulation requirement under the committed model and configuration. It does not establish hardware safety, robust stability across untested parameter ranges, or functional-safety compliance.
