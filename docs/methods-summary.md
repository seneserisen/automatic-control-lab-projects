# Methods summary

## Local linearisation

For a nonlinear system

\[
\dot{x}=f(x,u),
\]

a local model around an equilibrium \((x_0,u_0)\) is

\[
\Delta\dot{x}=A\Delta x+B\Delta u,
\quad
A=\left.\frac{\partial f}{\partial x}\right|_{x_0,u_0},
\quad
B=\left.\frac{\partial f}{\partial u}\right|_{x_0,u_0}.
\]

The resulting controller is normally valid only near the operating point.

## State feedback

A state-feedback law

\[
u=-Kx+N_r r
\]

changes the closed-loop dynamics to \(A-BK\). Pole placement and LQR are two common ways to choose \(K\).

## Fifth-order trajectory

A smooth point-to-point reference with zero initial and final velocity and acceleration is

\[
r(\tau)=r_0+(r_f-r_0)(10\tau^3-15\tau^4+6\tau^5),
\quad 0\le\tau\le1.
\]

Its derivatives can be used to calculate a model-based feedforward input.

## PI anti-windup

For a saturated actuator, back-calculation modifies the integral state:

\[
\dot{x}_I=e+K_{aw}(u_{sat}-u_{unsat}).
\]

The correction prevents the integrator from accumulating a large error while the actuator cannot follow the requested control signal.
