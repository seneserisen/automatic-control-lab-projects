#include "control_lab/two_tank.h"

#include <stdio.h>

static void print_case(const char *label, const two_tank_metrics_t *metrics)
{
    printf("%s_recovery_time_s=%.2f\n", label, metrics->recovery_time_s);
    printf("%s_peak_integral=%.6f\n", label, metrics->peak_integral_state);
    printf("%s_saturation_duration_s=%.2f\n", label, metrics->saturation_duration_s);
    printf("%s_final_level_2_cm=%.6f\n", label, metrics->final_level_2_cm);
}

int main(void)
{
    two_tank_config_t config;
    two_tank_metrics_t without_anti_windup;
    two_tank_metrics_t with_anti_windup;

    two_tank_default_config(&config);
    if (two_tank_run(&config, 0, &without_anti_windup) != 0
        || two_tank_run(&config, 1, &with_anti_windup) != 0) {
        fputs("Two-tank simulation failed.\n", stderr);
        return 1;
    }

    printf("initial_level_1_cm=%.6f\n", without_anti_windup.initial_level_1_cm);
    printf("initial_level_2_cm=%.6f\n", without_anti_windup.initial_level_2_cm);
    printf(
        "maximum_steady_state_level_cm=%.6f\n",
        without_anti_windup.maximum_steady_state_level_cm);
    print_case("without_anti_windup", &without_anti_windup);
    print_case("with_anti_windup", &with_anti_windup);
    printf(
        "recovery_improvement_s=%.2f\n",
        without_anti_windup.recovery_time_s
            - with_anti_windup.recovery_time_s);
    return 0;
}
