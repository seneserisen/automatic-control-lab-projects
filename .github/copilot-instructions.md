# Automatic Control Laboratory — Copilot Instructions

- Read `AGENTS.md` and the relevant files under `docs/` before substantial changes.
- Keep MATLAB, Python validation, and C99 runtime equations, units, state ordering, and parameter meaning aligned.
- Preserve attribution and distinguish original team laboratory work from independently structured portfolio implementations.
- Do not invent numerical results, citations, APIs, successful runs, or hardware claims.
- Use `docs/testing.md` for exact commands and run every affected language layer.
- Add behavior-focused tests for numerical, saturation, disturbance, invalid-input, and convergence changes.
- Do not weaken tolerances or expected values merely to make tests pass.
- Keep C99 portability and fixed-size/no-dynamic-allocation constraints where documented.
- Preserve unrelated work and avoid speculative controllers or broad rewrites.
- Update model assumptions, verification traceability, generated evidence, and project status when behavior changes.
- Report only checks that actually ran and review the complete diff.
