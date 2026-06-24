%NONLINEAR_CONTROL_DEMO Local linearisation and stabilisation of a nonlinear system.
%
% Clean-room portfolio reconstruction. The system is chosen so that it has
% equilibria at [0;0] and [1;1], with the second equilibrium locally unstable.

clear; close all; clc;
script_dir = fileparts(mfilename('fullpath'));
if isempty(script_dir), script_dir = pwd; end

x_eq_stable = [0; 0];
x_eq_unstable = [1; 1];
B = [0; 1];

A_stable = jacobian_at(x_eq_stable);
A_unstable = jacobian_at(x_eq_unstable);

fprintf('Stable-equilibrium Jacobian:\n'); disp(A_stable);
fprintf('Eigenvalues: '); disp(eig(A_stable).');
fprintf('Unstable-equilibrium Jacobian:\n'); disp(A_unstable);
fprintf('Eigenvalues: '); disp(eig(A_unstable).');

% Pole-placement result for A_unstable, B and desired poles [-2, -3].
K = [1, 3];
fprintf('Closed-loop eigenvalues near [1;1]: ');
disp(eig(A_unstable - B*K).');

t_span = [0, 8];
x0_stable = x_eq_stable + [0.18; -0.10];
x0_unstable = x_eq_unstable + [0.02; -0.02];

[t_s, x_s] = ode45(@(t, x) nonlinear_dynamics(t, x, 0), t_span, x0_stable);
[t_u, x_u] = ode45(@(t, x) nonlinear_dynamics(t, x, 0), [0, 2.5], x0_unstable);
[t_c, x_c] = ode45(@(t, x) nonlinear_dynamics(t, x, -K*(x - x_eq_unstable)), ...
                   t_span, x0_unstable);

% Linearised closed-loop response for comparison.
[t_l, dx_l] = ode45(@(t, dx) (A_unstable - B*K)*dx, t_span, x0_unstable - x_eq_unstable);
x_l = dx_l + x_eq_unstable.';

fig = figure('Color', 'w', 'Position', [100, 100, 1050, 780]);
subplot(2,2,1);
plot(t_s, x_s(:,1), 'LineWidth', 1.5); hold on;
plot(t_s, x_s(:,2), 'LineWidth', 1.5);
yline(0, '--k'); grid on;
title('Open-loop response near stable equilibrium');
xlabel('Time [s]'); ylabel('State'); legend('x_1','x_2','Location','best');

subplot(2,2,2);
plot(t_u, x_u(:,1), 'LineWidth', 1.5); hold on;
plot(t_u, x_u(:,2), 'LineWidth', 1.5);
yline(1, '--k'); grid on;
title('Open-loop response near unstable equilibrium');
xlabel('Time [s]'); ylabel('State'); legend('x_1','x_2','Location','best');

subplot(2,2,3);
plot(t_c, x_c(:,1), 'LineWidth', 1.5); hold on;
plot(t_c, x_c(:,2), 'LineWidth', 1.5);
yline(1, '--k'); grid on;
title('Nonlinear plant with local state feedback');
xlabel('Time [s]'); ylabel('State'); legend('x_1','x_2','Location','best');

subplot(2,2,4);
plot(t_c, x_c(:,1), 'LineWidth', 1.6); hold on;
plot(t_l, x_l(:,1), '--', 'LineWidth', 1.4);
yline(1, ':k'); grid on;
title('Nonlinear vs linearised closed-loop response');
xlabel('Time [s]'); ylabel('x_1'); legend('Nonlinear','Linearised','Equilibrium','Location','best');

sgtitle('Nonlinear control-loop analysis');
out_path = fullfile(script_dir, 'figures', 'nonlinear_control_results.png');
exportgraphics(fig, out_path, 'Resolution', 160);
fprintf('Saved figure: %s\n', out_path);

function dx = nonlinear_dynamics(~, x, u)
    % Two-state nonlinear system with equilibria at [0;0] and [1;1].
    dx = [
        -x(1)^3 + x(2);
        x(1) + x(2) - 2 + (1 - x(1))^2 + (1 - x(2))^2 + u
    ];
end

function A = jacobian_at(x)
    A = [
        -3*x(1)^2, 1;
        2*x(1) - 1, 2*x(2) - 1
    ];
end
