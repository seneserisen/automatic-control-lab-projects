"""Generate compact SVG previews and run numerical checks."""

from __future__ import annotations

import argparse
from html import escape
from pathlib import Path
from typing import Iterable

import numpy as np

from validation.reference_models import (
    active_suspension_simulation,
    magnetic_levitation_simulation,
    nonlinear_linearisation,
    repository_root,
    rotary_arm_simulation,
    two_tank_simulation,
)

WIDTH, HEIGHT = 900, 500
MARGIN_LEFT, MARGIN_RIGHT, MARGIN_TOP, MARGIN_BOTTOM = 75, 30, 55, 65


def _sample(x: np.ndarray, y: np.ndarray, maximum: int = 400) -> tuple[np.ndarray, np.ndarray]:
    step = max(1, int(np.ceil(x.size / maximum)))
    return x[::step], y[::step]


def _polyline(x: np.ndarray, y: np.ndarray, xlim: tuple[float, float], ylim: tuple[float, float]) -> str:
    plot_w = WIDTH - MARGIN_LEFT - MARGIN_RIGHT
    plot_h = HEIGHT - MARGIN_TOP - MARGIN_BOTTOM
    x_den = max(xlim[1] - xlim[0], 1e-12)
    y_den = max(ylim[1] - ylim[0], 1e-12)
    px = MARGIN_LEFT + (x - xlim[0]) / x_den * plot_w
    py = MARGIN_TOP + (ylim[1] - y) / y_den * plot_h
    return " ".join(f"{a:.1f},{b:.1f}" for a, b in zip(px, py, strict=True))


def _write_line_chart(
    path: Path,
    title: str,
    xlabel: str,
    ylabel: str,
    series: Iterable[tuple[str, np.ndarray, np.ndarray, str]],
) -> None:
    sampled = [(name, *_sample(x, y), colour) for name, x, y, colour in series]
    all_x = np.concatenate([x for _, x, _, _ in sampled])
    all_y = np.concatenate([y for _, _, y, _ in sampled])
    xlim = (float(np.min(all_x)), float(np.max(all_x)))
    ymin, ymax = float(np.min(all_y)), float(np.max(all_y))
    pad = max((ymax - ymin) * 0.08, 1e-6)
    ylim = (ymin - pad, ymax + pad)

    plot_w = WIDTH - MARGIN_LEFT - MARGIN_RIGHT
    plot_h = HEIGHT - MARGIN_TOP - MARGIN_BOTTOM
    elements = [
        f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {WIDTH} {HEIGHT}" role="img">',
        '<rect width="100%" height="100%" fill="white"/>',
        f'<text x="{WIDTH/2}" y="28" text-anchor="middle" font-family="Arial" font-size="22" font-weight="bold">{escape(title)}</text>',
        f'<rect x="{MARGIN_LEFT}" y="{MARGIN_TOP}" width="{plot_w}" height="{plot_h}" fill="none" stroke="#333"/>',
    ]
    for fraction in np.linspace(0, 1, 6):
        x = MARGIN_LEFT + fraction * plot_w
        y = MARGIN_TOP + fraction * plot_h
        elements.append(f'<line x1="{x:.1f}" y1="{MARGIN_TOP}" x2="{x:.1f}" y2="{MARGIN_TOP+plot_h}" stroke="#ddd"/>')
        elements.append(f'<line x1="{MARGIN_LEFT}" y1="{y:.1f}" x2="{MARGIN_LEFT+plot_w}" y2="{y:.1f}" stroke="#ddd"/>')
        xv = xlim[0] + fraction * (xlim[1] - xlim[0])
        yv = ylim[1] - fraction * (ylim[1] - ylim[0])
        elements.append(f'<text x="{x:.1f}" y="{MARGIN_TOP+plot_h+22}" text-anchor="middle" font-family="Arial" font-size="12">{xv:.2g}</text>')
        elements.append(f'<text x="{MARGIN_LEFT-10}" y="{y+4:.1f}" text-anchor="end" font-family="Arial" font-size="12">{yv:.2g}</text>')

    for name, x, y, colour in sampled:
        elements.append(f'<polyline points="{_polyline(x, y, xlim, ylim)}" fill="none" stroke="{colour}" stroke-width="2.2"/>')

    elements.append(f'<text x="{WIDTH/2}" y="{HEIGHT-14}" text-anchor="middle" font-family="Arial" font-size="14">{escape(xlabel)}</text>')
    elements.append(f'<text x="18" y="{HEIGHT/2}" text-anchor="middle" transform="rotate(-90 18 {HEIGHT/2})" font-family="Arial" font-size="14">{escape(ylabel)}</text>')
    legend_x = MARGIN_LEFT + 15
    legend_y = MARGIN_TOP + 20
    for idx, (name, _, _, colour) in enumerate(sampled):
        y = legend_y + idx * 22
        elements.append(f'<line x1="{legend_x}" y1="{y}" x2="{legend_x+28}" y2="{y}" stroke="{colour}" stroke-width="3"/>')
        elements.append(f'<text x="{legend_x+36}" y="{y+4}" font-family="Arial" font-size="13">{escape(name)}</text>')
    elements.append('</svg>')
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(elements), encoding="utf-8")


def save_nonlinear(root: Path) -> None:
    result = nonlinear_linearisation()
    values = [
        float(np.max(np.real(result.stable_eigenvalues))),
        float(np.max(np.real(result.unstable_eigenvalues))),
        float(np.max(np.real(result.closed_loop_eigenvalues))),
    ]
    labels = ["Stable equilibrium", "Unstable equilibrium", "Closed loop"]
    colours = ["#2a6fbb", "#c23b22", "#2b9348"]
    ymin, ymax = min(values) - 0.5, max(values) + 0.5
    plot_w = WIDTH - MARGIN_LEFT - MARGIN_RIGHT
    plot_h = HEIGHT - MARGIN_TOP - MARGIN_BOTTOM
    zero_y = MARGIN_TOP + (ymax / (ymax - ymin)) * plot_h
    bar_w = 130
    elements = [
        f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {WIDTH} {HEIGHT}">',
        '<rect width="100%" height="100%" fill="white"/>',
        f'<text x="{WIDTH/2}" y="30" text-anchor="middle" font-family="Arial" font-size="22" font-weight="bold">Local stability comparison</text>',
        f'<line x1="{MARGIN_LEFT}" y1="{zero_y:.1f}" x2="{WIDTH-MARGIN_RIGHT}" y2="{zero_y:.1f}" stroke="#333"/>',
    ]
    for idx, (label, value, colour) in enumerate(zip(labels, values, colours, strict=True)):
        cx = MARGIN_LEFT + plot_w * (idx + 0.5) / 3
        value_y = MARGIN_TOP + (ymax - value) / (ymax - ymin) * plot_h
        top, height = min(zero_y, value_y), abs(zero_y - value_y)
        elements.append(f'<rect x="{cx-bar_w/2:.1f}" y="{top:.1f}" width="{bar_w}" height="{height:.1f}" fill="{colour}"/>')
        elements.append(f'<text x="{cx:.1f}" y="{HEIGHT-45}" text-anchor="middle" font-family="Arial" font-size="13">{escape(label)}</text>')
        elements.append(f'<text x="{cx:.1f}" y="{value_y-8 if value >= 0 else value_y+18:.1f}" text-anchor="middle" font-family="Arial" font-size="14">{value:.2f}</text>')
    elements.append(f'<text x="20" y="{HEIGHT/2}" text-anchor="middle" transform="rotate(-90 20 {HEIGHT/2})" font-family="Arial" font-size="14">Maximum pole real part [1/s]</text>')
    elements.append('</svg>')
    path = root / "projects/nonlinear-control-loops/figures/nonlinear_control_results.svg"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(elements), encoding="utf-8")


def save_rotary(root: Path) -> None:
    result = rotary_arm_simulation()
    _write_line_chart(
        root / "projects/rotary-arm/figures/rotary_arm_results.svg",
        "Rotary-arm reference tracking",
        "Time [s]",
        "Angle [rad]",
        [("Reference", result.time, result.reference, "#c23b22"), ("Response", result.time, result.response, "#2a6fbb")],
    )


def save_suspension(root: Path) -> None:
    result = active_suspension_simulation()
    _write_line_chart(
        root / "projects/active-suspension/figures/active_suspension_results.svg",
        "Quarter-car body acceleration",
        "Time [s]",
        "Acceleration [m/s²]",
        [("Passive", result.time, result.passive_acceleration, "#c23b22"), ("Active", result.time, result.active_acceleration, "#2a6fbb")],
    )


def save_maglev(root: Path) -> None:
    result = magnetic_levitation_simulation()
    _write_line_chart(
        root / "projects/magnetic-levitation/figures/magnetic_levitation_results.svg",
        "Magnetic-levitation reference tracking",
        "Time [s]",
        "Gap deviation [mm]",
        [("Reference", result.time, 1000 * result.reference, "#c23b22"), ("Response", result.time, 1000 * result.position, "#2a6fbb")],
    )


def save_tanks(root: Path) -> None:
    result = two_tank_simulation()
    _write_line_chart(
        root / "projects/two-tank-system/figures/two_tank_results.svg",
        "Two-tank PI control and anti-windup",
        "Time [s]",
        "Tank-2 level [cm]",
        [
            ("Reference", result.time, result.reference, "#444444"),
            ("No anti-windup", result.time, result.no_anti_windup.h2, "#c23b22"),
            ("Back-calculation", result.time, result.anti_windup.h2, "#2a6fbb"),
        ],
    )


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
