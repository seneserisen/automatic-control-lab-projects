#include "control_lab/maglev_observer.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

static void require_condition(int condition, const char *message)
{
    if (condition == 0) {
        fprintf(stderr, "FAILED: %s\n", message);
        exit(EXIT_FAILURE);
    }
}

static int nearly_equal(double left, double right, double tolerance)
{
    return fabs(left - right) <= tolerance;
}

static void test_invalid_configuration(void)
{
    maglev_config_t config;
    maglev_metrics_t metrics;

    maglev_default_config(&config);
    config.sample_time_s = 0.0;
    require_condition(maglev_run(&config, &metrics) != 0, "invalid sample time must fail");
}

static void test_noise_free_tracking(void)
{
    maglev_config_t config;
    maglev_metrics_t metrics;

    maglev_default_config(&config);
    config.gap_noise_std_m = 0.0;
    config.current_noise_std_a = 0.0;
    require_condition(maglev_run(&config, &metrics) == 0, "noise-free simulation must run");
    require_condition(
        nearly_equal(metrics.final_position_m, config.reference_amplitude_m, 1.0e-5),
        "final position must track the reference");
    require_condition(
        metrics.position_estimation_rmse_m < 2.0e-6,
        "position estimation RMSE must remain bounded");
    require_condition(
        metrics.minimum_voltage_v >= config.voltage_min_v,
        "minimum voltage must respect saturation");
    require_condition(
        metrics.maximum_voltage_v <= config.voltage_max_v,
        "maximum voltage must respect saturation");
}

static void test_fixed_seed_is_deterministic(void)
{
    maglev_config_t config;
    maglev_metrics_t first;
    maglev_metrics_t second;

    maglev_default_config(&config);
    require_condition(maglev_run(&config, &first) == 0, "first seeded simulation must run");
    require_condition(maglev_run(&config, &second) == 0, "second seeded simulation must run");
    require_condition(first.final_position_m == second.final_position_m, "final position must be deterministic");
    require_condition(
        first.observer_tracking_rmse_m == second.observer_tracking_rmse_m,
        "tracking RMSE must be deterministic");
    require_condition(
        first.position_estimation_rmse_m == second.position_estimation_rmse_m,
        "position estimation RMSE must be deterministic");
    require_condition(first.maximum_voltage_v == second.maximum_voltage_v, "peak voltage must be deterministic");
}

static void test_fixed_step_convergence(void)
{
    maglev_config_t medium_config;
    maglev_config_t fine_config;
    maglev_metrics_t medium;
    maglev_metrics_t fine;

    maglev_default_config(&medium_config);
    medium_config.gap_noise_std_m = 0.0;
    medium_config.current_noise_std_a = 0.0;
    fine_config = medium_config;
    medium_config.sample_time_s = 1.0e-4;
    fine_config.sample_time_s = 5.0e-5;

    require_condition(maglev_run(&medium_config, &medium) == 0, "medium-step simulation must run");
    require_condition(maglev_run(&fine_config, &fine) == 0, "fine-step simulation must run");
    require_condition(
        fabs(medium.final_position_m - fine.final_position_m) < 1.0e-8,
        "final position must converge with step refinement");
    require_condition(
        fabs(medium.maximum_voltage_v - fine.maximum_voltage_v) < 1.0e-3,
        "peak voltage must converge with step refinement");
}

static void test_reference_metrics_are_reasonable(void)
{
    maglev_config_t config;
    maglev_metrics_t metrics;

    maglev_default_config(&config);
    require_condition(maglev_run(&config, &metrics) == 0, "noisy simulation must run");
    require_condition(metrics.observer_tracking_rmse_m < 2.5e-4, "tracking RMSE must remain below 0.25 mm");
    require_condition(metrics.position_estimation_rmse_m < 2.0e-6, "position estimation must remain accurate");
    require_condition(metrics.velocity_estimation_rmse_m_s < 2.0e-4, "velocity estimation must remain accurate");
    require_condition(metrics.current_estimation_rmse_a < 1.0e-3, "current estimation must remain accurate");
}

int main(void)
{
    test_invalid_configuration();
    test_noise_free_tracking();
    test_fixed_seed_is_deterministic();
    test_fixed_step_convergence();
    test_reference_metrics_are_reasonable();
    puts("All C magnetic-levitation tests passed.");
    return EXIT_SUCCESS;
}
