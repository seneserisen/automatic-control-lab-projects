function study = maglev_convergence_study(step_sizes)
%MAGLEV_CONVERGENCE_STUDY Compare fixed-step solutions against a fine reference.

arguments
    step_sizes (1,:) double {mustBePositive, mustBeFinite} = ...
        [8e-4, 4e-4, 2e-4, 1e-4, 5e-5]
end

configuration = maglev_configuration();
configuration.measurement.gap_noise_std = 0;
configuration.measurement.current_noise_std = 0;
model = maglev_linear_model(configuration);
design = design_maglev_observer(configuration, model);

tracking_rmse = zeros(size(step_sizes));
final_position = zeros(size(step_sizes));
maximum_voltage = zeros(size(step_sizes));

for idx = 1:numel(step_sizes)
    configuration.simulation.dt = step_sizes(idx);
    result = simulate_maglev_observer(configuration, model, design);
    metrics = calculate_maglev_observer_metrics(result, design);
    tracking_rmse(idx) = metrics.observer_tracking.rmse;
    final_position(idx) = result.observer_deviation(1,end);
    maximum_voltage(idx) = metrics.maximum_voltage;
end

study.step_sizes = step_sizes;
study.tracking_rmse = tracking_rmse;
study.final_position = final_position;
study.maximum_voltage = maximum_voltage;
study.final_position_difference_from_finest = ...
    abs(final_position-final_position(end));
study.voltage_difference_from_finest = ...
    abs(maximum_voltage-maximum_voltage(end));
end
