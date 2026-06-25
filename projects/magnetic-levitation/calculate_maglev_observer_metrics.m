function metrics = calculate_maglev_observer_metrics(result, design)
%CALCULATE_MAGLEV_OBSERVER_METRICS Summarize tracking and estimation quality.

arguments
    result (1,1) struct
    design (1,1) struct
end

metrics.full_state_tracking = control_lab.tracking_metrics( ...
    result.time, result.reference, result.full_state_deviation(1,:), ...
    result.full_state_voltage);
metrics.observer_tracking = control_lab.tracking_metrics( ...
    result.time, result.reference, result.observer_deviation(1,:), ...
    result.observer_voltage);
metrics.position_estimation_rmse = control_lab.rms_value( ...
    result.estimation_error(1,:));
metrics.velocity_estimation_rmse = control_lab.rms_value( ...
    result.estimation_error(2,:));
metrics.current_estimation_rmse = control_lab.rms_value( ...
    result.estimation_error(3,:));
metrics.maximum_voltage = max(result.observer_voltage);
metrics.minimum_voltage = min(result.observer_voltage);
metrics.observability_rank = design.observability_rank;
metrics.controller_poles = design.controller_poles;
metrics.observer_poles = design.observer_poles;
end
