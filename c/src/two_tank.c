#include "control_lab/two_tank.h"

#include <math.h>
#include <stddef.h>

#define TWO_TANK_STATE_DIM 2

static double clamp_value(double value, double lower, double upper)
{
    if (value < lower) {
        return lower;
    }
    if (value > upper) {
        return upper;
    }
    return value;
}

static void tank_derivative(
    const two_tank_config_t *config,
    const double state[TWO_TANK_STATE_DIM],
    double inlet_flow,
    double derivative[TWO_TANK_STATE_DIM])
{
    double level_1 = state[0] > 0.0 ? state[0] : 0.0;
    double level_2 = state[1] > 0.0 ? state[1] : 0.0;
    double level_difference = level_1 - level_2;
    double flow_12;
    double flow_out;

    if (level_difference < 0.0) {
        level_difference = 0.0;
    }

    flow_12 = config->outlet_12_cm2
        * sqrt(2.0 * config->gravity_cm_s2 * level_difference);
    flow_out = config->outlet_2_cm2
        * sqrt(2.0 * config->gravity_cm_s2 * level_2);

    derivative[0] = (inlet_flow - flow_12) / config->area_1_cm2;
    derivative[1] = (flow_12 - flow_out) / config->area_2_cm2;
}

static void rk4_step(
    const two_tank_config_t *config,
    const double state[TWO_TANK_STATE_DIM],
    double inlet_flow,
    double next_state[TWO_TANK_STATE_DIM])
{
    double k1[TWO_TANK_STATE_DIM];
    double k2[TWO_TANK_STATE_DIM];
    double k3[TWO_TANK_STATE_DIM];
    double k4[TWO_TANK_STATE_DIM];
    double temporary[TWO_TANK_STATE_DIM];
    size_t index;
    const double dt = config->sample_time_s;

    tank_derivative(config, state, inlet_flow, k1);
    for (index = 0U; index < TWO_TANK_STATE_DIM; ++index) {
        temporary[index] = state[index] + 0.5 * dt * k1[index];
    }
    tank_derivative(config, temporary, inlet_flow, k2);
    for (index = 0U; index < TWO_TANK_STATE_DIM; ++index) {
        temporary[index] = state[index] + 0.5 * dt * k2[index];
    }
    tank_derivative(config, temporary, inlet_flow, k3);
    for (index = 0U; index < TWO_TANK_STATE_DIM; ++index) {
        temporary[index] = state[index] + dt * k3[index];
    }
    tank_derivative(config, temporary, inlet_flow, k4);

    for (index = 0U; index < TWO_TANK_STATE_DIM; ++index) {
        next_state[index] = state[index]
            + dt * (k1[index] + 2.0 * k2[index] + 2.0 * k3[index] + k4[index]) / 6.0;
        if (next_state[index] < 0.0) {
            next_state[index] = 0.0;
        }
    }
}

void two_tank_default_config(two_tank_config_t *config)
{
    if (config == NULL) {
        return;
    }

    config->area_1_cm2 = 50.0;
    config->area_2_cm2 = 50.0;
    config->outlet_12_cm2 = 0.20;
    config->outlet_2_cm2 = 0.18;
    config->gravity_cm_s2 = 981.0;

    config->kp = 5.0;
    config->ki = 0.05;
    config->anti_windup_gain = 0.30;
    config->pump_min_cm3_s = 0.0;
    config->pump_max_cm3_s = 45.0;

    config->sample_time_s = 0.05;
    config->final_time_s = 700.0;
    config->initial_level_2_cm = 10.0;

    config->high_reference_cm = 35.0;
    config->high_reference_start_s = 100.0;
    config->high_reference_end_s = 160.0;

    config->disturbance_cm3_s = -5.0;
    config->disturbance_start_s = 520.0;
    config->disturbance_end_s = 580.0;

    config->recovery_tolerance_cm = 1.0;
    config->recovery_hold_time_s = 20.0;
    config->recovery_window_start_s = 160.0;
    config->recovery_window_end_s = 480.0;
}

int two_tank_validate_config(const two_tank_config_t *config)
{
    if (config == NULL) {
        return -1;
    }
    if (!(config->area_1_cm2 > 0.0) || !(config->area_2_cm2 > 0.0)
        || !(config->outlet_12_cm2 > 0.0) || !(config->outlet_2_cm2 > 0.0)
        || !(config->gravity_cm_s2 > 0.0)) {
        return -2;
    }
    if (!(config->ki > 0.0) || config->kp < 0.0
        || config->anti_windup_gain < 0.0) {
        return -3;
    }
    if (!(config->pump_max_cm3_s > config->pump_min_cm3_s)) {
        return -4;
    }
    if (!(config->sample_time_s > 0.0)
        || !(config->final_time_s > config->sample_time_s)) {
        return -5;
    }
    if (config->high_reference_start_s < 0.0
        || config->high_reference_end_s <= config->high_reference_start_s
        || config->high_reference_end_s >= config->final_time_s) {
        return -6;
    }
    if (config->disturbance_start_s < 0.0
        || config->disturbance_end_s <= config->disturbance_start_s
        || config->disturbance_end_s > config->final_time_s) {
        return -7;
    }
    if (!(config->recovery_tolerance_cm > 0.0)
        || !(config->recovery_hold_time_s > 0.0)
        || config->recovery_window_start_s < config->high_reference_end_s
        || config->recovery_window_end_s <= config->recovery_window_start_s
        || config->recovery_window_end_s > config->final_time_s) {
        return -8;
    }
    return 0;
}

int two_tank_run(
    const two_tank_config_t *config,
    int use_anti_windup,
    two_tank_metrics_t *metrics)
{
    double state[TWO_TANK_STATE_DIM];
    double next_state[TWO_TANK_STATE_DIM];
    double steady_flow;
    double integral_state;
    double minimum_control;
    double maximum_control;
    double peak_integral;
    double saturation_duration = 0.0;
    double recovery_time = NAN;
    unsigned long inside_count = 0UL;
    unsigned long hold_samples;
    unsigned long steps;
    unsigned long step;

    if (metrics == NULL || two_tank_validate_config(config) != 0) {
        return -1;
    }

    steady_flow = config->outlet_2_cm2
        * sqrt(2.0 * config->gravity_cm_s2 * config->initial_level_2_cm);
    state[1] = config->initial_level_2_cm;
    state[0] = state[1]
        + pow(steady_flow / config->outlet_12_cm2, 2.0)
            / (2.0 * config->gravity_cm_s2);
    integral_state = steady_flow / config->ki;
    minimum_control = config->pump_max_cm3_s;
    maximum_control = config->pump_min_cm3_s;
    peak_integral = fabs(integral_state);
    steps = (unsigned long)llround(config->final_time_s / config->sample_time_s);
    hold_samples = (unsigned long)ceil(
        config->recovery_hold_time_s / config->sample_time_s);

    for (step = 0UL; step <= steps; ++step) {
        const double time_s = (double)step * config->sample_time_s;
        const double reference =
            (time_s >= config->high_reference_start_s
                && time_s < config->high_reference_end_s)
                ? config->high_reference_cm
                : config->initial_level_2_cm;
        const double disturbance =
            (time_s >= config->disturbance_start_s
                && time_s < config->disturbance_end_s)
                ? config->disturbance_cm3_s
                : 0.0;
        const double error = reference - state[1];
        const double unsaturated = config->kp * error
            + config->ki * integral_state;
        const double control = clamp_value(
            unsaturated,
            config->pump_min_cm3_s,
            config->pump_max_cm3_s);
        const double correction = use_anti_windup != 0
            ? config->anti_windup_gain * (control - unsaturated)
            : 0.0;

        if (control < minimum_control) {
            minimum_control = control;
        }
        if (control > maximum_control) {
            maximum_control = control;
        }
        if (fabs(control - unsaturated) > 1.0e-12) {
            saturation_duration += config->sample_time_s;
        }
        if (fabs(integral_state) > peak_integral) {
            peak_integral = fabs(integral_state);
        }

        if (time_s >= config->recovery_window_start_s
            && time_s <= config->recovery_window_end_s
            && fabs(state[1] - config->initial_level_2_cm)
                <= config->recovery_tolerance_cm) {
            inside_count += 1UL;
            if (inside_count >= hold_samples && isnan(recovery_time)) {
                recovery_time = time_s
                    - (double)(hold_samples - 1UL) * config->sample_time_s;
            }
        } else {
            inside_count = 0UL;
        }

        if (step == steps) {
            break;
        }

        integral_state += config->sample_time_s * (error + correction);
        rk4_step(config, state, control + disturbance, next_state);
        state[0] = next_state[0];
        state[1] = next_state[1];

        if (!isfinite(state[0]) || !isfinite(state[1])
            || !isfinite(integral_state)) {
            return -2;
        }
    }

    metrics->initial_level_1_cm = config->initial_level_2_cm
        + pow(steady_flow / config->outlet_12_cm2, 2.0)
            / (2.0 * config->gravity_cm_s2);
    metrics->initial_level_2_cm = config->initial_level_2_cm;
    metrics->maximum_steady_state_level_cm = pow(
        config->pump_max_cm3_s / config->outlet_2_cm2,
        2.0) / (2.0 * config->gravity_cm_s2);
    metrics->final_level_1_cm = state[0];
    metrics->final_level_2_cm = state[1];
    metrics->recovery_time_s = recovery_time;
    metrics->peak_integral_state = peak_integral;
    metrics->minimum_control_cm3_s = minimum_control;
    metrics->maximum_control_cm3_s = maximum_control;
    metrics->saturation_duration_s = saturation_duration;
    metrics->sample_count = steps + 1UL;
    return 0;
}
