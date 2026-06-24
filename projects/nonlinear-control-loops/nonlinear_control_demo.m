%NONLINEAR_CONTROL_DEMO Local linearisation and stabilisation of a nonlinear system.
%
% Clean-room portfolio reconstruction. The system is chosen so that it has
% equilibria at [0;0] and [1;1], with the second equilibrium locally unstable.

clear; close all; clc;
script_dir = fileparts(mfilename('fullpath'));
if isempty(script_dir), script_dir = pwd; end
repo_root = fileparts(fileparts(script_dir));
addpath(fullfile(repo_root, 'matlab'));

x_eq_stable = [0; 0];
x_eq_unstable = [1; 1];
B = [0; 1];

A_stable = jacobian_at(x_eq_stable);
A_unstable = jacobian_at(x_eq_unstable);
controllability_rank = rank([B, A_unstable*B]);

fprintf('Stable-equilibrium Jacobian:\n'); disp(A_stable);
fprintf('Eigenvalues: '); disp(eig(A_stable).');
fprintf('Unstable-equilibrium Jacobian:\n'); disp(A_unstable);
fprintf('Eigenvalues: '); disp(eig(A_unstable).');
fprintf('Controllability rank at [1;1]: %d\n', controllability_rank);

% Pole-placement result for A_unstable, B and desired poles [-2, -3].
K = [1, 3];
A_closed = A_unstable - B*K;
fprintf('Closed-loop eigenvalues near [1;1]: ');
disp(eig(A_closed).');

t_eval = linspace(0, 4, 801);
near_offset = [0.02; -0.02];
far_offset = [0.25; -0.20];

[near_nonlinear, near_linear] = compare_models(t_eval, x_eq_unstable, near_offset, K, A_closed);
[far_nonlinear, far_linear] = compare_models(t_eval, x_eq_unstable, far_offset, K, A_closed);
near_rmse = control_lab.rms_value(near_nonlinear(:,1) - near_linear(:,1));
far_rmse = control_lab.rms_value(far_nonlinear(:,1) - far_linear(:,1));

[t_stable, x_stable] = ode45(@(time, state) nonlinear_dynamics(time, state, 0), ...
    [0, 8], x_eq_stable + [0.18; -0.10]);
[t_unstable, x_unstable] = ode45(@(time, state) nonlinear_dynamics(time, state, 0), ...
    [0, 2.5], x_eq_unstable + near_offset);

fig = figure('Color', 'w', 'Position', [100, 100, 1100, 780]);
subplot(2,2,1);
plot(t_stable, x_stable(:,1), 'LineWidth', 1.5); hold on;
plot(t_stable, x_stable(:,2), 'LineWidth', 1.5);
yline(0, '--k'); grid on;
title('Open loop near stable equilibrium');
xlabel('Time [s]'); ylabel('State'); legend('x_1','x_2','Location','best');

subplot(2,2,2);
plot(t_unstable, x_unstable(:,1), 'LineWidth', 1.5); hold on;
plot(t_unstable, x_unstable(:,2), 'LineWidth', 1.5);
yline(1, '--k'); grid on;
title('Open loop near unstable equilibrium');
xlabel('Time [s]'); ylabel('State'); legend('x_1','x_2','Location','best');

subplot(2,2,3);
plot(t_eval, near_nonlinear(:,1), 'LineWidth', 1.6); hold on;
plot(t_eval, near_linear(:,1), '--', 'LineWidth', 1.4);
yline(1, ':k'); grid on;
title(sprintf('Near equilibrium: RMSE %.4g', near_rmse));
xlabel('Time [s]'); ylabel('x_1'); legend('Nonlinear','Linearised','Equilibrium','Location','best');

subplot(2,2,4);
plot(t_eval, far_nonlinear(:,1), 'LineWidth', 1.6); hold on;
plot(t_eval, far_linear(:,1), '--', 'LineWidth', 1.4);
yline(1, ':k'); grid on;
title(sprintf('Farther away: RMSE %.4g', far_rmse));
xlabel('Time [s]'); ylabel('x_1'); legend('Nonlinear','Linearised','Equilibrium','Location','best');

sgtitle('Nonlinear control-loop analysis');
out_path = fullfile(script_dir, 'figures', 'nonlinear_control_results.png');
exportgraphics(fig, out_path, 'Resolution', 160);
fprintf('Near-point linearisation RMSE: %.6f\n', near_rmse);
fprintf('Farther-point linearisation RMSE: %.6f\n', far_rmse);
fprintf('Saved figure: %s\n', out_path);

function [nonlinear_response, linear_response] = compare_models(time, equilibrium, offset, gain, A_closed)
    [~, nonlinear_response] = ode45(...
        @(current_time, state) nonlinear_dynamics(current_time, state, -gain*(state-equilibrium)), ...
        time, equilibrium + offset);
    [~, linear_deviation] = ode45(@(~, deviation) A_closed*deviation, time, offset);
    linear_response = linear_deviation + equilibrium.';
end

function derivative = nonlinear_dynamics(~, state, control)
    derivative = [
        -state(1)^3 + state(2);
        state(1) + state(2) - 2 + (1-state(1))^2 + (1-state(2))^2 + control
    ];
end

function A = jacobian_at(state)
    A = [
        -3*state(1)^2, 1;
        2*state(1)-1, 2*state(2)-1
    ];
end
