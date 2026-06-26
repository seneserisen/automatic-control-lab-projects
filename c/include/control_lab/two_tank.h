#ifndef CONTROL_LAB_TWO_TANK_H
#define CONTROL_LAB_TWO_TANK_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    double area_1_cm2;
    double area_2_cm2;
    double outlet_12_cm2;
    double outlet_2_cm2;
    double gravity_cm_s2;

    double kp;
    double ki;
    double anti_windup_gain;
    double pump_min_cm3_s;
    double pump_max_cm3_s;

    double sample_time_s;
    double final_time_s;
    double initial_level_2_cm;

    double high_reference_cm;
    double high_reference_start_s;
    double high_reference_end_s;

    double disturbance_cm3_s;
    double disturbance_start_s;
    double disturbance_end_s;

    double recovery_tolerance_cm;
    double recovery_hold_time_s;
    double recovery_window_start_s;
    double recovery_window_end_s;
} two_tank_config_t;

typedef struct {
    double initial_level_1_cm;
    double initial_level_2_cm;
    double maximum_steady_state_level_cm;
    double final_level_1_cm;
    double final_level_2_cm;
    double recovery_time_s;
    double peak_integral_state;
    double minimum_control_cm3_s;
    double maximum_control_cm3_s;
    double saturation_duration_s;
    unsigned long sample_count;
} two_tank_metrics_t;

void two_tank_default_config(two_tank_config_t *config);
int two_tank_validate_config(const two_tank_config_t *config);
int two_tank_run(
    const two_tank_config_t *config,
    int use_anti_windup,
    two_tank_metrics_t *metrics);

#ifdef __cplusplus
}
#endif

#endif
