function result = simulate_maglev_observer(configuration, model, design)
%SIMULATE_MAGLEV_OBSERVER Compare full-state and observer-based control.

arguments
    configuration (1,1) struct
    model (1,1) struct
    design (1,1) struct
end

step_size = configuration.simulation.dt;
time = 0:step_size:configuration.simulation.final_time;
reference = zeros(size(time));
reference(time >= configuration.reference.start_time) = ...
    configuration.reference.amplitude;

rng(configuration.measurement.seed, 'twister');
gap_noise = configuration.measurement.gap_noise_std*randn(size(time));
current_noise = configuration.measurement.current_noise_std*randn(size(time));

observer_plant = zeros(3, numel(time));
observer_plant(:,1) = model.equilibrium_state;
full_state_plant = zeros(3, numel(time));
full_state_plant(:,1) = model.equilibrium_state;
estimated_state = zeros(3, numel(time));
measurements = zeros(2, numel(time));
observer_voltage = zeros(size(time));
full_state_voltage = zeros(size(time));

for idx = 1:numel(time)-1
    true_deviation = observer_plant(:,idx)-model.equilibrium_state;
    measurements(:,idx) = model.C*true_deviation + ...
        [gap_noise(idx); current_noise(idx)];

    observer_voltage_deviation = ...
        -design.controller_gain*estimated_state(:,idx) + ...
        design.prefilter*reference(idx);
    observer_voltage(idx) = control_lab.saturate( ...
        model.equilibrium_voltage+observer_voltage_deviation, ...
        configuration.voltage.minimum, configuration.voltage.maximum);
    applied_voltage_deviation = observer_voltage(idx)-model.equilibrium_voltage;

    plant_rhs = @(~, state) maglev_nonlinear_dynamics( ...
        state, observer_voltage(idx), configuration);
    observer_rhs = @(~, estimate) model.A*estimate + ...
        model.B*applied_voltage_deviation + ...
        design.observer_gain*(measurements(:,idx)-model.C*estimate);

    observer_plant(:,idx+1) = control_lab.rk4_step( ...
        plant_rhs, time(idx), observer_plant(:,idx), step_size);
    estimated_state(:,idx+1) = control_lab.rk4_step( ...
        observer_rhs, time(idx), estimated_state(:,idx), step_size);

    full_deviation = full_state_plant(:,idx)-model.equilibrium_state;
    full_voltage_deviation = ...
        -design.controller_gain*full_deviation + ...
        design.prefilter*reference(idx);
    full_state_voltage(idx) = control_lab.saturate( ...
        model.equilibrium_voltage+full_voltage_deviation, ...
        configuration.voltage.minimum, configuration.voltage.maximum);
    full_rhs = @(~, state) maglev_nonlinear_dynamics( ...
        state, full_state_voltage(idx), configuration);
    full_state_plant(:,idx+1) = control_lab.rk4_step( ...
        full_rhs, time(idx), full_state_plant(:,idx), step_size);
end

observer_voltage(end) = observer_voltage(end-1);
full_state_voltage(end) = full_state_voltage(end-1);
final_deviation = observer_plant(:,end)-model.equilibrium_state;
measurements(:,end) = model.C*final_deviation + ...
    [gap_noise(end); current_noise(end)];

result.time = time;
result.reference = reference;
result.observer_plant = observer_plant;
result.full_state_plant = full_state_plant;
result.estimated_state = estimated_state;
result.measurements = measurements;
result.observer_voltage = observer_voltage;
result.full_state_voltage = full_state_voltage;
result.gap_noise = gap_noise;
result.current_noise = current_noise;
result.observer_deviation = observer_plant-model.equilibrium_state;
result.full_state_deviation = full_state_plant-model.equilibrium_state;
result.estimation_error = result.observer_deviation-estimated_state;
end
