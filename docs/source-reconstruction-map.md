# Source-document reconstruction map

The original laboratory material consisted of team reports, MATLAB scripts, Simulink models, preparation tasks, experiment instructions, figures, and result discussions. Those documents are treated as **source specifications**, not as files to upload unchanged.

The public repository reconstructs the engineering content into original code, tests, explanations, and reproducible evidence while preserving four-person team attribution and excluding raw university material.

## Transformation rule

Each source document should produce the following public artifacts:

1. **Problem statement** — the physical system and control objective in original wording.
2. **Model** — equations, states, inputs, outputs, parameters, and assumptions.
3. **Implementation** — clean MATLAB functions and, where deployment value is high, a C or C++ runtime.
4. **Validation** — automated tests, numerical checks, disturbances, limits, and convergence evidence.
5. **Results** — generated metrics and representative plots.
6. **Interpretation** — what succeeded, what failed, and why.
7. **Limitations** — what the simulation cannot prove.
8. **Attribution** — explicit statement that the historical exercise was completed in a four-person team.

Raw reports, university instructions, screenshots, and copied team submissions are not published.

## Current reconstruction status

| Source project | MATLAB reconstruction | Independent validation | Deployment-oriented implementation | Public documentation |
|---|---|---|---|---|
| Nonlinear control loops | Implemented | Python reference and tests | Not yet needed | Implemented |
| Elastically mounted rotary arm | Implemented | Python reference and tests | Planned only if a reusable motion-control runtime is defined | Implemented |
| Quarter-car active suspension | Implemented | Python reference and tests | Planned as a discrete state-feedback example | Implemented |
| Magnetic levitation | Modular MATLAB implementation and direct MATLAB tests | Python observer and convergence reference | Portable C99 observer runtime implemented | Implemented |
| Two-tank process | Implemented | Python reference and tests | Planned as an embedded PI and anti-windup runtime | Implemented |

## Why not convert every document directly into C

The source documents do not all describe software intended for embedded deployment. Some primarily demonstrate analysis, linearisation, LQR design, or simulation comparison. Rewriting every equation in C would create code volume without adding engineering value.

C or C++ is added when at least one of the following applies:

- the algorithm is intended to execute cyclically on a controller;
- deterministic memory and timing behaviour matter;
- the implementation demonstrates embedded-software architecture;
- hardware-in-the-loop or real-time validation is a credible next step;
- the code is reusable in later robotics, automotive, or industrial-control projects.

MATLAB remains the correct environment for design and analysis. C/C++ is the correct next layer for deployable control logic. Python remains an independent reference and automation layer.

## Next reconstruction priorities

1. Convert the two-tank PI and anti-windup controller into a portable C runtime.
2. Add a discrete active-suspension state-feedback implementation in C.
3. Add document-to-code traceability sections to every project README.
4. Record which equations and parameters came from the historical team work and which public implementation decisions were newly reconstructed.
5. Add explicit cross-language tolerance checks for deterministic, noise-free scenarios.
