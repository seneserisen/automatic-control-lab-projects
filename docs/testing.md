# Testing and Verification

Use the commands that apply to the changed layer. Do not treat one language layer as proof that another layer works.

## Python validation

```bash
python -m venv .venv
source .venv/bin/activate  # Windows: .venv\Scripts\activate
pip install -r requirements.txt
ruff check .
ruff format --check .
pytest -q
python -m validation.generate_reference_figures --check-only
python -m validation.report --check
```

Python validation should cover independent numerical references, behavior, generated-report freshness, invalid inputs, and regression cases.

## MATLAB validation

From MATLAB R2024b or the documented compatible environment:

```matlab
run_all
results = runtests('matlab/tests', 'IncludeSubfolders', true);
assertSuccess(results);
```

Record the MATLAB version and any toolbox requirements when results depend on them.

## Portable C validation

```bash
cmake -S c -B build/c -DCMAKE_BUILD_TYPE=Release
cmake --build build/c --parallel
ctest --test-dir build/c --output-on-failure
./build/c/maglev_observer_demo
./build/c/two_tank_demo
```

Where supported, validate with both GCC and Clang and keep warnings treated as errors.

## Required numerical checks

For affected models, verify relevant:

- equilibrium and operating point;
- dimensions and state ordering;
- controllability and observability;
- eigenvalues or pole placement;
- zero-input and zero-disturbance behavior;
- saturation and anti-windup behavior;
- observer convergence and noise response;
- integration-step convergence;
- deterministic repeatability;
- invalid parameter handling;
- cross-language agreement within a justified tolerance.

## Evidence review

When a change affects published results:

1. regenerate the relevant output;
2. compare against independently calculated or cross-language evidence;
3. update `docs/results-summary.md` only with reproduced values;
4. update `docs/verification-matrix.md` when requirements or tests change;
5. inspect generated figures for labels, units, and misleading scales.

## CI boundaries

Repository workflows validate Python, MATLAB, and C separately. Passing CI does not establish hardware behavior, real-time scheduling, fixed-point behavior, robust stability outside tested ranges, MISRA compliance, or functional safety.

## Baseline failures

Record any failure that existed before a task in `docs/project_status.md` before attributing it to new work.
