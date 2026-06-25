#include "control_lab/maglev_observer.h"

#include <stdio.h>

int main(void)
{
    maglev_config_t config;
    maglev_metrics_t metrics;

    maglev_default_config(&config);
    if (maglev_run(&config, &metrics) != 0) {
        fputs("Magnetic-levitation simulation failed.\n", stderr);
        return 1;
    }

    printf("samples=%lu\n", metrics.sample_count);
    printf("final_position_mm=%.9f\n", 1000.0 * metrics.final_position_m);
    printf("tracking_rmse_mm=%.9f\n", 1000.0 * metrics.observer_tracking_rmse_m);
    printf("position_estimation_rmse_mm=%.9f\n", 1000.0 * metrics.position_estimation_rmse_m);
    printf("velocity_estimation_rmse_m_s=%.9g\n", metrics.velocity_estimation_rmse_m_s);
    printf("current_estimation_rmse_a=%.9g\n", metrics.current_estimation_rmse_a);
    printf("minimum_voltage_v=%.6f\n", metrics.minimum_voltage_v);
    printf("maximum_voltage_v=%.6f\n", metrics.maximum_voltage_v);
    return 0;
}
