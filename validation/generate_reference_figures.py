"""Generate preview figures and run basic numerical checks."""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from validation.reference_models import (
    active_suspension_simulation,
    magnetic_levitation_simulation,
    nonlinear_linearisation,
    repository_root,
    rotary_arm_simulation,
    two_tank_simulation,
)


def save_rotary(root: Path) -> None:
    result = rotary_arm_simulation()
    fig, axes = plt.subplots(2, 1, figsize=(9, 7), constrained_layout=True)
    axes[0].plot(result.time, result.reference, "--", label="Reference")
    axes[0].plot(result.time, result.response, label="Response")
    axes[0].set(xlabel="Time [s]", ylabel="Angle [rad]", title="Rotary-arm tracking")
    axes[0].grid(True)
    axes[0].legend()
    axes[1].plot(result.time, result.feedforward, "--", label="Feedforward")
    axes[1].plot(result.time, result.control, label="Total control")
    axes[1].set(xlabel="Time [s]", ylabel="Input", title="Control effort")
    axes[1].grid(True)
    axes[1].legend()
    fig.savefig(root / "projects/rotary-arm/figures/rotary_arm_results.svg")
    plt.close(fig)


def save_suspension(root: Path) -> None:
    result = active_suspension_simulation()
    fig, axes = plt.subplots(2, 1, figsize=(9, 7), constrained_layout=True)
    axes[0].plot(result.time, 1000 * result.road, "--", label="Road")
    axes[0].plot(result.time, 1000 * result.passive_body, label="Passive")
    axes[0].plot(result.time, 1000 * result.active_body, label="Active")
    axes[0].set(xlabel="Time [s]", ylabel="Displacement [mm]", title="Body displacement")
    axes[0].grid(True)
    axes[0].legend()
    axes[1].plot(result.time, result.passive_acceleration, label="Passive")
    axes[1].plot(result.time, result.active_acceleration, label="Active")
    axes[1].set(xlabel="Time [s]", ylabel="Acceleration [m/s²]", title="Body acceleration")
    axes[1].grid(True)
    axes[1].legend()
    fig.savefig(root / "projects/active-suspension/figures/active_suspension_results.svg")
    plt.close(fig)


def save_maglev(root: Path) -> None:
    result = magnetic_levitation_simulation()
    fig, axes = plt.subplots(2, 1, figsize=(9, 7), constrained_layout=True)
    axes[0].plot(result.time, 1000 * result.reference, "--", label="Reference")
    axes[0].plot(result.time, 1000 * result.position, label="Position")
    axes[0].set(xlabel="Time [s]", ylabel="Gap deviation [mm]", title="Magnetic levitation tracking")
    axes[0].grid(True)
    axes[0].legend()
    axes[1].plot(result.time, result.voltage, label="Voltage deviation")
    axes[1].set(xlabel="Time [s]", ylabel="Voltage [V]", title="Control input")
    axes[1].grid(True)
    axes[1].legend()
    fig.savefig(root / "projects/magnetic-levitation/figures/magnetic_levitation_results.svg")
    plt.close(fig)


def save_tanks(root: Path) -> None:
    result = two_tank_simulation()
    fig, axes = plt.subplots(2, 1, figsize=(9, 7), constrained_layout=True)
    axes[0].plot(result.time, result.reference, "--", label="Reference")
    axes[0].plot(result.time, result.no_anti_windup.h2, label="No anti-windup")
    axes[0].plot(result.time, result.anti_windup.h2, label="Back-calculation")
    axes[0].set(xlabel="Time [s]", ylabel="Tank-2 level [cm]", title="Two-tank level control")
    axes[0].grid(True)
    axes[0].legend()
    axes[1].plot(result.time, result.no_anti_windup.integral_state, label="No anti-windup")
    axes[1].plot(result.time, result.anti_windup.integral_state, label="Back-calculation")
    axes[1].set(xlabel="Time [s]", ylabel="Integral state", title="Integrator windup")
    axes[1].grid(True)
    axes[1].legend()
    fig.savefig(root / "projects/two-tank-system/figures/two_tank_results.svg")
    plt.close(fig)


def save_nonlinear(root: Path) -> None:
    result = nonlinear_linearisation()
    labels = ["Stable equilibrium", "Unstable equilibrium", "Closed loop"]
    max_real = [
        np.max(np.real(result.stable_eigenvalues)),
        np.max(np.real(result.unstable_eigenvalues)),
        np.max(np.real(result.closed_loop_eigenvalues)),
    ]
    fig, ax = plt.subplots(figsize=(8, 5), constrained_layout=True)
    ax.bar(labels, max_real)
    ax.axhline(0.0, linewidth=1.0)
    ax.set(ylabel="Maximum pole real part [1/s]", title="Local stability comparison")
    ax.grid(True, axis="y")
    fig.savefig(root / "projects/nonlinear-control-loops/figures/nonlinear_control_results.svg")
    plt.close(fig)


def numerical_checks() -> None:
    nonlinear = nonlinear_linearisation()
    assert np.max(np.real(nonlinear.stable_eigenvalues)) < 0
    assert np.max(np.real(nonlinear.unstable_eigenvalues)) > 0
    assert np.max(np.real(nonlinear.closed_loop_eigenvalues)) < 0

    rotary = rotary_arm_simulation()
    assert abs(rotary.response[-1] - 1.0) < 1e-3
    assert np.max(np.abs(rotary.control)) <= 25.0 + 1e-12

    suspension = active_suspension_simulation()
    assert np.max(np.abs(suspension.active_acceleration)) < np.max(np.abs(suspension.passive_acceleration))

    maglev = magnetic_levitation_simulation()
    assert abs(maglev.equilibrium_current - 2.0011) < 5e-3
    assert np.max(np.real(maglev.open_loop_poles)) > 0
    assert np.max(np.real(maglev.closed_loop_poles)) < 0
    assert abs(maglev.position[-1] - 0.001) < 2e-5

    tanks = two_tank_simulation()
    after_return = tanks.time >= 300.0
    no_aw_error = np.abs(tanks.no_anti_windup.h2[after_return] - 10.0)
    aw_error = np.abs(tanks.anti_windup.h2[after_return] - 10.0)
    no_aw_recovery = tanks.time[after_return][np.where(no_aw_error < 1.0)[0][0]]
    aw_recovery = tanks.time[after_return][np.where(aw_error < 1.0)[0][0]]
    assert aw_recovery < no_aw_recovery


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--check-only", action="store_true")
    args = parser.parse_args()

    numerical_checks()
    if args.check_only:
        print("Reference-model checks passed.")
        return

    root = repository_root()
    save_nonlinear(root)
    save_rotary(root)
    save_suspension(root)
    save_maglev(root)
    save_tanks(root)
    print("Reference figures generated.")


if __name__ == "__main__":
    main()
