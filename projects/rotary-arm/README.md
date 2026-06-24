# Elastically Mounted Rotary Arm

This reconstruction focuses on smooth point-to-point motion and the division of responsibility between feedforward and feedback control.

## What the script does

- models a simplified third-order arm plant;
- calculates a fifth-order reference trajectory with zero initial/final velocity and acceleration;
- derives the feedforward input from the model and trajectory derivatives;
- adds stabilising state feedback;
- applies an actuator limit;
- plots the plant frequency response without requiring Control System Toolbox.

## Why fifth order?

A fifth-order polynomial supplies six coefficients, enough to enforce position, velocity and acceleration boundary conditions at the beginning and end of the move.

## Run

```matlab
rotary_arm_demo
```

## Engineering interpretation

Feedforward creates the nominal motion required by the model. Feedback corrects modelling error and disturbances. Increasing the transition time reduces peak velocity, acceleration, jerk and feedforward effort.

## Preview

![Rotary Arm results](figures/rotary_arm_results.svg)
