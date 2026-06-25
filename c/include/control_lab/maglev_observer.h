#ifndef CONTROL_LAB_MAGLEV_OBSERVER_H
#define CONTROL_LAB_MAGLEV_OBSERVER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    double mass_kg;
    double gravity_m_s2;
    double equilibrium_gap_m;
    double magnetic_coefficient;
    double resistance_ohm;
    double inductance_h;

    double controller_gain[3];
    double observer_gain[3][2];
    double reference_prefilter;

    double sample_time_s;
    double final_time_s;
    double reference_start_s;
    double reference_amplitude_m;

    double voltage_min_v;
    double voltage_max_v;
    double gap_noise_std_m;
    double current_noise_std_a;
    uint32_t noise_seed;
} maglev_config_t;

typedef struct {
    double final_position_m;
    double final_velocity_m_s;
    double final_current_a;
    double observer_tracking_rmse_m;
    double position_estimation_rmse_m;
    double velocity_estimation_rmse_m_s;
    double current_estimation_rmse_a;
    double minimum_voltage_v;
    double maximum_voltage_v;
    unsigned long sample_count;
} maglev_metrics_t;

void maglev_default_config(maglev_config_t *config);
int maglev_validate_config(const maglev_config_t *config);
int maglev_run(const maglev_config_t *config, maglev_metrics_t *metrics);

#ifdef __cplusplus
}
#endif

#endif
