# Contributing

Contributions that improve reproducibility, numerical validation, documentation, portability, or test coverage are welcome.

## Suitable contributions

- Correcting equations, units, terminology, or documentation
- Adding numerical regression tests
- Improving MATLAB, Python, CMake, or C99 portability
- Adding deterministic examples with clearly documented assumptions
- Improving requirements-to-test traceability
- Reporting reproducible defects with the smallest useful example

## Development workflow

1. Open an issue describing the problem, expected behaviour, and proposed scope.
2. Create a focused branch from `main`.
3. Keep each pull request limited to one coherent change.
4. Add or update tests for behavioural changes.
5. Run the relevant validation commands before opening the pull request.

### Python validation

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

### C validation

```bash
cmake -S c -B build/c -DCMAKE_BUILD_TYPE=Release
cmake --build build/c --parallel
ctest --test-dir build/c --output-on-failure
```

### MATLAB validation

```matlab
run_all
results = runtests('matlab/tests', 'IncludeSubfolders', true);
assertSuccess(results);
```

## Engineering expectations

- State physical assumptions and operating ranges.
- Distinguish measured results from theoretical expectations.
- Do not claim hardware, real-time, safety, or production readiness without corresponding evidence.
- Use SI units or state conversions explicitly.
- Prefer deterministic tests and fixed random seeds.
- Explain numerical tolerances rather than choosing them only to make tests pass.
- Avoid committing generated build directories or local environment files.

## Academic integrity

Do not submit copied course solutions, private team reports, examination material, or restricted university documents. Contributions must be independently written and suitable for public release.
