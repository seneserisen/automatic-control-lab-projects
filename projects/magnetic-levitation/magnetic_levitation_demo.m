%MAGNETIC_LEVITATION_DEMO Run the modular observer-based maglev experiment.

clear; close all; clc;
script_dir = fileparts(mfilename('fullpath'));
if isempty(script_dir), script_dir = pwd; end
repo_root = fileparts(fileparts(script_dir));
addpath(fullfile(repo_root, 'matlab'));
addpath(script_dir);

configuration = maglev_configuration();
model = maglev_linear_model(configuration);
design = design_maglev_observer(configuration, model);
result = simulate_maglev_observer(configuration, model, design);
metrics = calculate_maglev_observer_metrics(result, design);
convergence = maglev_convergence_study();

fprintf('Gain source: %s\n', design.gain_source);
fprintf('Equilibrium current: %.4f A\n', model.equilibrium_current);
fprintf('Equilibrium voltage: %.4f V\n', model.equilibrium_voltage);
fprintf('Observability rank: %d\n', design.observability_rank);
fprintf('Controller poles:\n'); disp(design.controller_poles.');
fprintf('Observer poles:\n'); disp(design.observer_poles.');
fprintf('Full-state tracking RMSE: %.6f mm\n', ...
    1000*metrics.full_state_tracking.rmse);
fprintf('Observer tracking RMSE: %.6f mm\n', ...
    1000*metrics.observer_tracking.rmse);
fprintf('Position-estimation RMSE: %.6f mm\n', ...
    1000*metrics.position_estimation_rmse);
fprintf('Velocity-estimation RMSE: %.6g m/s\n', ...
    metrics.velocity_estimation_rmse);
fprintf('Current-estimation RMSE: %.6g A\n', ...
    metrics.current_estimation_rmse);
fprintf('Maximum applied voltage: %.4f V\n', metrics.maximum_voltage);
fprintf('0.1 ms vs 0.05 ms final-position difference: %.6g mm\n', ...
    1000*convergence.final_position_difference_from_finest(end-1));

output_path = fullfile(script_dir, 'figures', ...
    'magnetic_levitation_observer_results.png');
plot_maglev_observer_results(result, metrics, output_path);
fprintf('Saved figure: %s\n', output_path);
