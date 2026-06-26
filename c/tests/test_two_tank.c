#include "control_lab/two_tank.h"

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

static void test_invalid_configuration(void)
{
    two_tank_config_t config;
    two_tank_metrics_t metrics;

    two_tank_default_config(&config);
    config.sample_time_s = 0.0;
    require_condition(
        two_tank_run(&config, 1, &metrics) != 0,
        "invalid sample time must be rejected");
}

static void test_physical_equilibrium_and_pump_limit(void)
{
    two_tank_config_t config;
    two_tank_metrics_t metrics;

    two_tank_default_config(&config);
    require_condition(
        two_tank_run(&config, 1, &metrics) == 0,
        "anti-windup simulation must run");
    require_condition(
        fabs(metrics.initial_level_1_cm - 18.1) < 1.0e-9,
        "initial tank-1 level must satisfy equilibrium");
    require_condition(
        fabs(metrics.initial_level_2_cm - 10.0) < 1.0e-12,
        "initial tank-2 level must match the configured equilibrium");
    require_condition(
        metrics.maximum_steady_state_level_cm < config.high_reference_cm,
        "high reference must exceed the pump-supported steady-state level");
}

static void test_anti_windup_improves_recovery(void)
{
    two_tank_config_t config;
    two_tank_metrics_t without_anti_windup;
    two_tank_metrics_t with_anti_windup;

    two_tank_default_config(&config);
    require_condition(
        two_tank_run(&config, 0, &without_anti_windup) == 0,
        "standard PI simulation must run");
    require_condition(
        two_tank_run(&config, 1, &with_anti_windup) == 0,
        "anti-windup PI simulation must run");

    require_condition(
        fabs(without_anti_windup.recovery_time_s - 428.65) < 0.1,
        "standard PI recovery must match the reference result");
    require_condition(
        fabs(with_anti_windup.recovery_time_s - 311.65) < 0.1,
        "anti-windup recovery must match the reference result");
    require_condition(
        with_anti_windup.recovery_time_s
            < without_anti_windup.recovery_time_s - 100.0,
        "back-calculation must improve recovery by more than 100 seconds");
    require_condition(
        with_anti_windup.peak_integral_state
            < 0.5 * without_anti_windup.peak_integral_state,
        "back-calculation must substantially limit integrator growth");
}

static void test_control_limits_and_final_state(void)
{
    two_tank_config_t config;
    two_tank_metrics_t metrics;

    two_tank_default_config(&config);
    require_condition(
        two_tank_run(&config, 1, &metrics) == 0,
        "anti-windup simulation must run");
    require_condition(
        metrics.minimum_control_cm3_s >= config.pump_min_cm3_s,
        "minimum pump command must respect saturation");
    require_condition(
        metrics.maximum_control_cm3_s <= config.pump_max_cm3_s,
        "maximum pump command must respect saturation");
    require_condition(
        metrics.saturation_duration_s > 0.0,
        "unreachable reference must cause actuator saturation");
    require_condition(
        fabs(metrics.final_level_2_cm - config.initial_level_2_cm) < 0.5,
        "tank-2 level must return close to the nominal operating level");
}

static void test_fixed_step_convergence(void)
{
    two_tank_config_t coarse_config;
    two_tank_config_t fine_config;
    two_tank_metrics_t coarse;
    two_tank_metrics_t fine;

    two_tank_default_config(&coarse_config);
    fine_config = coarse_config;
    coarse_config.sample_time_s = 0.10;
    fine_config.sample_time_s = 0.05;

    require_condition(
        two_tank_run(&coarse_config, 1, &coarse) == 0,
        "coarse-step simulation must run");
    require_condition(
        two_tank_run(&fine_config, 1, &fine) == 0,
        "fine-step simulation must run");
    require_condition(
        fabs(coarse.recovery_time_s - fine.recovery_time_s) <= 0.1,
        "recovery time must remain stable under step refinement");
    require_condition(
        fabs(coarse.final_level_2_cm - fine.final_level_2_cm) < 1.0e-4,
        "final tank level must converge under step refinement");
}

int main(void)
{
    test_invalid_configuration();
    test_physical_equilibrium_and_pump_limit();
    test_anti_windup_improves_recovery();
    test_control_limits_and_final_state();
    test_fixed_step_convergence();
    puts("All C two-tank tests passed.");
    return EXIT_SUCCESS;
}
