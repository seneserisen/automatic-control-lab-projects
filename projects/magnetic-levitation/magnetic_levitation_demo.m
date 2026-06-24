%MAGNETIC_LEVITATION_DEMO Compare local linear and nonlinear maglev models.

clear; close all; clc;
script_dir = fileparts(mfilename('fullpath'));
if isempty(script_dir), script_dir = pwd; end
repo_root = fileparts(fileparts(script_dir));
addpath(fullfile(repo_root, 'matlab'));

parameters.mass = 0.068;
parameters.gravity = 9.81;
parameters.gap = 0.014;
parameters.magnetic_coefficient = 6.53e-5;
parameters.resistance = 11.0;
parameters.inductance = 0.4125;

equilibrium_current = sqrt(2*parameters.mass*parameters.gravity*parameters.gap^2/parameters.magnetic_coefficient);
equilibrium_voltage = parameters.resistance*equilibrium_current;
position_gradient = 2*parameters.gravity/parameters.gap;
current_gradient = -parameters.magnetic_coefficient*equilibrium_current/(parameters.mass*parameters.gap^2);

A = [
    0, 1, 0;
    position_gradient, 0, current_gradient;
    0, 0, -parameters.resistance/parameters.inductance
];
B = [0; 0; 1/parameters.inductance];

% Pole-placement result for desired poles [-20,-30,-40].
gain = [-6316.60910998, -168.35876027, 26.125];
prefilter = -1009.79192166;
fprintf('Equilibrium current: %.4f A\n', equilibrium_current);
fprintf('Equilibrium voltage: %.4f V\n', equilibrium_voltage);
fprintf('Open-loop poles:\n'); disp(eig(A).');
fprintf('Closed-loop poles:\n'); disp(eig(A-B*gain).');

dt = 0.0001;
time = 0:dt:0.5;
reference = zeros(size(time));
reference(time >= 0.05) = 0.0005;  % 0.5 mm deviation
linear_state = zeros(3, numel(time));
nonlinear_state = zeros(3, numel(time));
nonlinear_state(:,1) = [parameters.gap; 0; equilibrium_current];
total_voltage = zeros(size(time));

for idx = 1:numel(time)-1
    equilibrium_state = [parameters.gap; 0; equilibrium_current];
    deviation = nonlinear_state(:,idx)-equilibrium_state;
    voltage_deviation = -gain*deviation+prefilter*reference(idx);
    total_voltage(idx) = control_lab.saturate(equilibrium_voltage+voltage_deviation, 0, 30);
    applied_deviation = total_voltage(idx)-equilibrium_voltage;

    linear_rhs = @(~, state) A*state+B*applied_deviation;
    nonlinear_rhs = @(~, state) maglev_dynamics(state, total_voltage(idx), parameters);
    linear_state(:,idx+1) = control_lab.rk4_step(linear_rhs, time(idx), linear_state(:,idx), dt);
    nonlinear_state(:,idx+1) = control_lab.rk4_step(nonlinear_rhs, time(idx), nonlinear_state(:,idx), dt);
end
total_voltage(end) = total_voltage(end-1);
nonlinear_deviation = nonlinear_state(1,:)-parameters.gap;
model_rmse = control_lab.rms_value(linear_state(1,:)-nonlinear_deviation);

fig = figure('Color', 'w', 'Position', [100, 100, 1100, 780]);
subplot(2,2,1);
plot(time, 1000*reference, 'k--', 'LineWidth', 1.4); hold on;
plot(time, 1000*linear_state(1,:), 'LineWidth', 1.4);
plot(time, 1000*nonlinear_deviation, 'LineWidth', 1.5); grid on;
title('Air-gap reference tracking'); xlabel('Time [s]'); ylabel('Deviation [mm]');
legend('Reference','Linear model','Nonlinear plant','Location','best');

subplot(2,2,2);
plot(time, nonlinear_state(3,:), 'LineWidth', 1.5); hold on;
yline(equilibrium_current, '--k'); grid on;
title('Coil current'); xlabel('Time [s]'); ylabel('A');

subplot(2,2,3);
plot(time, total_voltage, 'LineWidth', 1.5); hold on;
yline(equilibrium_voltage, '--k'); yline(30, ':k'); grid on;
title('Applied voltage'); xlabel('Time [s]'); ylabel('V');

subplot(2,2,4);
plot(time, 1000*(linear_state(1,:)-nonlinear_deviation), 'LineWidth', 1.4); grid on;
title(sprintf('Linear/nonlinear error: RMSE %.4g mm', 1000*model_rmse));
xlabel('Time [s]'); ylabel('Position error [mm]');

sgtitle('Magnetic levitation local-model validation');
out_path = fullfile(script_dir, 'figures', 'magnetic_levitation_results.png');
exportgraphics(fig, out_path, 'Resolution', 160);
fprintf('Linear/nonlinear position RMSE: %.6f mm\n', 1000*model_rmse);
fprintf('Maximum applied voltage: %.4f V\n', max(total_voltage));
fprintf('Saved figure: %s\n', out_path);

function derivative = maglev_dynamics(state, voltage, p)
    gap = max(state(1), 0.004);
    current = max(state(3), 0);
    magnetic_force = p.magnetic_coefficient*current^2/(2*gap^2);
    derivative = [
        state(2);
        p.gravity-magnetic_force/p.mass;
        (voltage-p.resistance*state(3))/p.inductance
    ];
end
