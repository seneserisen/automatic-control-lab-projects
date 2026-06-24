# Automatic Control Laboratory Projects

A curated control-engineering portfolio containing five reproducible MATLAB projects and licence-free Python reference simulations.

The projects reconstruct the main engineering ideas from a four-person **Automatic Control I** laboratory portfolio at FAU Erlangen-Nürnberg. The public repository is intentionally rebuilt from first principles rather than publishing raw team submissions or university material.

## Projects

| Project | Main engineering methods | MATLAB entry point |
|---|---|---|
| [Nonlinear control loops](projects/nonlinear-control-loops/) | Jacobian linearisation, equilibrium stability, local state feedback | `nonlinear_control_demo.m` |
| [Elastically mounted rotary arm](projects/rotary-arm/) | Bode analysis, fifth-order trajectory generation, feedforward, 2-DOF control | `rotary_arm_demo.m` |
| [Quarter-car active suspension](projects/active-suspension/) | State-space modelling, road disturbance, LQR/state feedback | `active_suspension_demo.m` |
| [Magnetic levitation](projects/magnetic-levitation/) | Nonlinear equilibrium, unstable linear model, state-feedback stabilisation | `magnetic_levitation_demo.m` |
| [Two-tank process](projects/two-tank-system/) | Nonlinear hydraulics, PI control, saturation, back-calculation anti-windup | `two_tank_demo.m` |

## Preview results

| Active suspension | Magnetic levitation | Two-tank anti-windup |
|---|---|---|
| ![Active suspension](projects/active-suspension/figures/active_suspension_results.svg) | ![Magnetic levitation](projects/magnetic-levitation/figures/magnetic_levitation_results.svg) | ![Two-tank](projects/two-tank-system/figures/two_tank_results.svg) |

## Technical coverage

- Nonlinear differential-equation modelling
- Operating-point and Jacobian linearisation
- State-space and transfer-function reasoning
- Pole and eigenvalue analysis
- Frequency-response interpretation
- P, PI, PID, state-feedback, and LQR concepts
- Smooth reference trajectories and feedforward control
- Disturbance compensation
- Actuator saturation and anti-windup

## Repository design

The MATLAB scripts are the main portfolio implementations. A Python reference layer mirrors the mathematical models to:

- produce preview figures without requiring a MATLAB licence in CI;
- validate important physical and control assumptions;
- keep the public repository reproducible on standard GitHub runners.

This does **not** replace MATLAB/Simulink validation. It provides a second, transparent implementation of the same equations.

## Quick start

### MATLAB

From the repository root:

```matlab
run_all
```

Or run an individual project:

```matlab
run('projects/magnetic-levitation/magnetic_levitation_demo.m')
```

Requirements:

- MATLAB R2021b or later recommended
- No Simulink model files are required for the script demonstrations
- Control System Toolbox is optional; precomputed stabilising gains are included where appropriate

### Python reference simulations

```bash
python -m venv .venv
source .venv/bin/activate  # Windows: .venv\Scripts\activate
pip install -r requirements.txt
python -m validation.generate_reference_figures
pytest
```

## Results and limitations

The models are educational and intentionally compact. They demonstrate control-design reasoning, not production-ready plant fidelity.

Important limitations include:

- simplified parameters and sensors;
- no real-time execution guarantees;
- no hardware-in-the-loop validation;
- no actuator, measurement, or safety certification;
- no claim that the clean-room code is identical to the original team submission.

## Academic attribution

The historical laboratory exercises were completed in a four-person team. See [NOTICE.md](NOTICE.md) and the [team contribution template](docs/team-contributions-template.md).
