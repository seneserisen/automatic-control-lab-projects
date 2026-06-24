# Nonlinear Control-Loop Analysis

This project demonstrates how a nonlinear model can have different local behaviour at different equilibrium points.

## What the script does

- defines a nonlinear two-state plant with equilibria at `[0, 0]` and `[1, 1]`;
- derives the Jacobian matrices analytically;
- compares the local eigenvalues;
- simulates open-loop behaviour near both equilibria;
- applies local state feedback around the unstable equilibrium;
- compares the nonlinear closed-loop response with the linearised prediction.

## Model

\[
\dot{x}_1=-x_1^3+x_2
\]

\[
\dot{x}_2=x_1+x_2-2+(1-x_1)^2+(1-x_2)^2+u
\]

At `[1,1]`, the Jacobian is

\[
A=\begin{bmatrix}-3&1\\1&1\end{bmatrix},\qquad
B=\begin{bmatrix}0\\1\end{bmatrix}.
\]

The open-loop equilibrium is unstable. The clean-room reconstruction uses `K = [1, 3]`, which places the local closed-loop poles at `-2` and `-3`.

## Run

```matlab
nonlinear_control_demo
```

## Engineering interpretation

Linearisation is local. The closer the initial state and disturbances remain to the chosen equilibrium, the more accurately the linear model predicts the nonlinear response.

## Preview

![Nonlinear Control Loops results](figures/nonlinear_control_results.svg)
