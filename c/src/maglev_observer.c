#include "control_lab/maglev_observer.h"

#include <math.h>
#include <stddef.h>

#define MAGLEV_STATE_DIM 3
#define MAGLEV_OUTPUT_DIM 2
#define MAGLEV_PI 3.14159265358979323846

typedef struct {
    uint32_t state;
    int has_spare;
    double spare;
} normal_rng_t;

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

static uint32_t xorshift32(uint32_t *state)
{
    uint32_t x = *state;
    if (x == 0U) {
        x = 0x6D2B79F5U;
    }
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    *state = x;
    return x;
}

static double uniform_open01(uint32_t *state)
{
    return ((double)xorshift32(state) + 0.5) / 4294967296.0;
}

static double normal_sample(normal_rng_t *rng)
{
    double radius;
    double angle;

    if (rng->has_spare != 0) {
        rng->has_spare = 0;
        return rng->spare;
    }

    radius = sqrt(-2.0 * log(uniform_open01(&rng->state)));
    angle = 2.0 * MAGLEV_PI * uniform_open01(&rng->state);
    rng->spare = radius * sin(angle);
    rng->has_spare = 1;
    return radius * cos(angle);
}

static void build_linear_model(
    const maglev_config_t *config,
    double equilibrium_current,
    double a[MAGLEV_STATE_DIM][MAGLEV_STATE_DIM],
    double b[MAGLEV_STATE_DIM])
{
    size_t row;
    size_t column;

    for (row = 0U; row < MAGLEV_STATE_DIM; ++row) {
        b[row] = 0.0;
        for (column = 0U; column < MAGLEV_STATE_DIM; ++column) {
            a[row][column] = 0.0;
        }
    }

    a[0][1] = 1.0;
    a[1][0] = 2.0 * config->gravity_m_s2 / config->equilibrium_gap_m;
    a[1][2] = -config->magnetic_coefficient * equilibrium_current
        / (config->mass_kg * config->equilibrium_gap_m * config->equilibrium_gap_m);
    a[2][2] = -config->resistance_ohm / config->inductance_h;
    b[2] = 1.0 / config->inductance_h;
}

static void plant_derivative(
    const maglev_config_t *config,
    const double state[MAGLEV_STATE_DIM],
    double voltage,
    double derivative[MAGLEV_STATE_DIM])
{
    double gap = state[0];
    double current = state[2];
    double magnetic_force;

    if (gap < 0.004) {
        gap = 0.004;
    }
    if (current < 0.0) {
        current = 0.0;
    }

    magnetic_force = config->magnetic_coefficient * current * current / (2.0 * gap * gap);
    derivative[0] = state[1];
    derivative[1] = config->gravity_m_s2 - magnetic_force / config->mass_kg;
    derivative[2] = (voltage - config->resistance_ohm * state[2]) / config->inductance_h;
}

static void observer_derivative(
    const maglev_config_t *config,
    double a[MAGLEV_STATE_DIM][MAGLEV_STATE_DIM],
    double b[MAGLEV_STATE_DIM],
    const double estimate[MAGLEV_STATE_DIM],
    double voltage_deviation,
    const double measurement[MAGLEV_OUTPUT_DIM],
    double derivative[MAGLEV_STATE_DIM])
{
    double innovation[MAGLEV_OUTPUT_DIM];
    size_t row;
    size_t column;

    innovation[0] = measurement[0] - estimate[0];
    innovation[1] = measurement[1] - estimate[2];

    for (row = 0U; row < MAGLEV_STATE_DIM; ++row) {
        double value = b[row] * voltage_deviation;
        for (column = 0U; column < MAGLEV_STATE_DIM; ++column) {
            value += a[row][column] * estimate[column];
        }
        value += config->observer_gain[row][0] * innovation[0];
        value += config->observer_gain[row][1] * innovation[1];
        derivative[row] = value;
    }
}

static void rk4_plant_step(
    const maglev_config_t *config,
    const double state[MAGLEV_STATE_DIM],
    double voltage,
    double next_state[MAGLEV_STATE_DIM])
{
    double k1[MAGLEV_STATE_DIM];
    double k2[MAGLEV_STATE_DIM];
    double k3[MAGLEV_STATE_DIM];
    double k4[MAGLEV_STATE_DIM];
    double temporary[MAGLEV_STATE_DIM];
    size_t index;
    const double dt = config->sample_time_s;

    plant_derivative(config, state, voltage, k1);
    for (index = 0U; index < MAGLEV_STATE_DIM; ++index) {
        temporary[index] = state[index] + 0.5 * dt * k1[index];
    }
    plant_derivative(config, temporary, voltage, k2);
    for (index = 0U; index < MAGLEV_STATE_DIM; ++index) {
        temporary[index] = state[index] + 0.5 * dt * k2[index];
    }
    plant_derivative(config, temporary, voltage, k3);
    for (index = 0U; index < MAGLEV_STATE_DIM; ++index) {
        temporary[index] = state[index] + dt * k3[index];
    }
    plant_derivative(config, temporary, voltage, k4);

    for (index = 0U; index < MAGLEV_STATE_DIM; ++index) {
        next_state[index] = state[index]
            + dt * (k1[index] + 2.0 * k2[index] + 2.0 * k3[index] + k4[index]) / 6.0;
    }
}

static void rk4_observer_step(
    const maglev_config_t *config,
    double a[MAGLEV_STATE_DIM][MAGLEV_STATE_DIM],
    double b[MAGLEV_STATE_DIM],
    const double estimate[MAGLEV_STATE_DIM],
    double voltage_deviation,
    const double measurement[MAGLEV_OUTPUT_DIM],
    double next_estimate[MAGLEV_STATE_DIM])
{
    double k1[MAGLEV_STATE_DIM];
    double k2[MAGLEV_STATE_DIM];
    double k3[MAGLEV_STATE_DIM];
    double k4[MAGLEV_STATE_DIM];
    double temporary[MAGLEV_STATE_DIM];
    size_t index;
    const double dt = config->sample_time_s;

    observer_derivative(config, a, b, estimate, voltage_deviation, measurement, k1);
    for (index = 0U; index < MAGLEV_STATE_DIM; ++index) {
        temporary[index] = estimate[index] + 0.5 * dt * k1[index];
    }
    observer_derivative(config, a, b, temporary, voltage_deviation, measurement, k2);
    for (index = 0U; index < MAGLEV_STATE_DIM; ++index) {
        temporary[index] = estimate[index] + 0.5 * dt * k2[index];
    }
    observer_derivative(config, a, b, temporary, voltage_deviation, measurement, k3);
    for (index = 0U; index < MAGLEV_STATE_DIM; ++index) {
        temporary[index] = estimate[index] + dt * k3[index];
    }
    observer_derivative(config, a, b, temporary, voltage_deviation, measurement, k4);

    for (index = 0U; index < MAGLEV_STATE_DIM; ++index) {
        next_estimate[index] = estimate[index]
            + dt * (k1[index] + 2.0 * k2[index] + 2.0 * k3[index] + k4[index]) / 6.0;
    }
}

void maglev_default_config(maglev_config_t *config)
{
    if (config == NULL) {
        return;
    }

    config->mass_kg = 0.068;
    config->gravity_m_s2 = 9.81;
    config->equilibrium_gap_m = 0.014;
    config->magnetic_coefficient = 6.53e-5;
    config->resistance_ohm = 11.0;
    config->inductance_h = 0.4125;

    config->controller_gain[0] = -6316.60910998;
    config->controller_gain[1] = -168.35876027;
    config->controller_gain[2] = 26.125;

    config->observer_gain[0][0] = 180.000004;
    config->observer_gain[0][1] = 0.0927619903;
    config->observer_gain[1][0] = 9401.42904;
    config->observer_gain[1][1] = -1.10085054;
    config->observer_gain[2][0] = 0.00114506735;
    config->observer_gain[2][1] = 63.3333293;

    config->reference_prefilter = -1009.79192166;
    config->sample_time_s = 1.0e-4;
    config->final_time_s = 0.5;
    config->reference_start_s = 0.05;
    config->reference_amplitude_m = 5.0e-4;
    config->voltage_min_v = 0.0;
    config->voltage_max_v = 30.0;
    config->gap_noise_std_m = 5.0e-6;
    config->current_noise_std_a = 2.0e-3;
    config->noise_seed = 42U;
}

int maglev_validate_config(const maglev_config_t *config)
{
    if (config == NULL) {
        return -1;
    }
    if (!(config->mass_kg > 0.0) || !(config->gravity_m_s2 > 0.0)
        || !(config->equilibrium_gap_m > 0.0) || !(config->magnetic_coefficient > 0.0)
        || !(config->resistance_ohm > 0.0) || !(config->inductance_h > 0.0)) {
        return -2;
    }
    if (!(config->sample_time_s > 0.0) || !(config->final_time_s > config->sample_time_s)
        || config->reference_start_s < 0.0
        || config->reference_start_s >= config->final_time_s) {
        return -3;
    }
    if (!(config->voltage_max_v > config->voltage_min_v)
        || config->gap_noise_std_m < 0.0 || config->current_noise_std_a < 0.0) {
        return -4;
    }
    return 0;
}

int maglev_run(const maglev_config_t *config, maglev_metrics_t *metrics)
{
    double plant[MAGLEV_STATE_DIM];
    double next_plant[MAGLEV_STATE_DIM];
    double estimate[MAGLEV_STATE_DIM] = {0.0, 0.0, 0.0};
    double next_estimate[MAGLEV_STATE_DIM];
    double a[MAGLEV_STATE_DIM][MAGLEV_STATE_DIM];
    double b[MAGLEV_STATE_DIM];
    double measurement[MAGLEV_OUTPUT_DIM];
    double equilibrium_current;
    double equilibrium_voltage;
    double sum_tracking_sq = 0.0;
    double sum_position_estimation_sq = 0.0;
    double sum_velocity_estimation_sq = 0.0;
    double sum_current_estimation_sq = 0.0;
    double minimum_voltage;
    double maximum_voltage;
    unsigned long step;
    unsigned long steps;
    normal_rng_t rng;

    if (metrics == NULL || maglev_validate_config(config) != 0) {
        return -1;
    }

    equilibrium_current = sqrt(
        2.0 * config->mass_kg * config->gravity_m_s2
        * config->equilibrium_gap_m * config->equilibrium_gap_m
        / config->magnetic_coefficient);
    equilibrium_voltage = config->resistance_ohm * equilibrium_current;
    build_linear_model(config, equilibrium_current, a, b);

    plant[0] = config->equilibrium_gap_m;
    plant[1] = 0.0;
    plant[2] = equilibrium_current;
    minimum_voltage = config->voltage_max_v;
    maximum_voltage = config->voltage_min_v;
    steps = (unsigned long)llround(config->final_time_s / config->sample_time_s);
    rng.state = config->noise_seed;
    rng.has_spare = 0;
    rng.spare = 0.0;

    for (step = 0UL; step <= steps; ++step) {
        const double time_s = (double)step * config->sample_time_s;
        const double reference = time_s >= config->reference_start_s
            ? config->reference_amplitude_m
            : 0.0;
        const double position_deviation = plant[0] - config->equilibrium_gap_m;
        const double current_deviation = plant[2] - equilibrium_current;
        const double gap_noise = config->gap_noise_std_m * normal_sample(&rng);
        const double current_noise = config->current_noise_std_a * normal_sample(&rng);
        const double tracking_error = reference - position_deviation;
        const double position_estimation_error = position_deviation - estimate[0];
        const double velocity_estimation_error = plant[1] - estimate[1];
        const double current_estimation_error = current_deviation - estimate[2];

        measurement[0] = position_deviation + gap_noise;
        measurement[1] = current_deviation + current_noise;
        sum_tracking_sq += tracking_error * tracking_error;
        sum_position_estimation_sq += position_estimation_error * position_estimation_error;
        sum_velocity_estimation_sq += velocity_estimation_error * velocity_estimation_error;
        sum_current_estimation_sq += current_estimation_error * current_estimation_error;

        if (step == steps) {
            break;
        }

        {
            const double voltage_deviation =
                -config->controller_gain[0] * estimate[0]
                -config->controller_gain[1] * estimate[1]
                -config->controller_gain[2] * estimate[2]
                + config->reference_prefilter * reference;
            const double voltage = clamp_value(
                equilibrium_voltage + voltage_deviation,
                config->voltage_min_v,
                config->voltage_max_v);

            if (voltage < minimum_voltage) {
                minimum_voltage = voltage;
            }
            if (voltage > maximum_voltage) {
                maximum_voltage = voltage;
            }

            rk4_plant_step(config, plant, voltage, next_plant);
            rk4_observer_step(
                config,
                a,
                b,
                estimate,
                voltage - equilibrium_voltage,
                measurement,
                next_estimate);
        }

        plant[0] = next_plant[0];
        plant[1] = next_plant[1];
        plant[2] = next_plant[2];
        estimate[0] = next_estimate[0];
        estimate[1] = next_estimate[1];
        estimate[2] = next_estimate[2];

        if (!isfinite(plant[0]) || !isfinite(plant[1]) || !isfinite(plant[2])
            || !isfinite(estimate[0]) || !isfinite(estimate[1]) || !isfinite(estimate[2])) {
            return -2;
        }
    }

    metrics->sample_count = steps + 1UL;
    metrics->final_position_m = plant[0] - config->equilibrium_gap_m;
    metrics->final_velocity_m_s = plant[1];
    metrics->final_current_a = plant[2];
    metrics->observer_tracking_rmse_m = sqrt(sum_tracking_sq / (double)metrics->sample_count);
    metrics->position_estimation_rmse_m = sqrt(
        sum_position_estimation_sq / (double)metrics->sample_count);
    metrics->velocity_estimation_rmse_m_s = sqrt(
        sum_velocity_estimation_sq / (double)metrics->sample_count);
    metrics->current_estimation_rmse_a = sqrt(
        sum_current_estimation_sq / (double)metrics->sample_count);
    metrics->minimum_voltage_v = minimum_voltage;
    metrics->maximum_voltage_v = maximum_voltage;
    return 0;
}
