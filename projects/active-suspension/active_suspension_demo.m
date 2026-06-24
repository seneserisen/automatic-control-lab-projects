%ACTIVE_SUSPENSION_DEMO Quarter-car state-space simulation.

clear; close all; clc;
script_dir = fileparts(mfilename('fullpath'));
if isempty(script_dir), script_dir = pwd; end

% Parameters from the laboratory-scale quarter-car model.
ms = 2.45;    % sprung mass [kg]
mu = 1.40;    % unsprung mass [kg]
ks = 900;     % suspension stiffness [N/m]
kt = 2500;    % tire stiffness [N/m]
cs = 12;      % suspension damping [N*s/m]
ct = 10;      % tire damping [N*s/m]

A = [
    0,       1,              0,             0;
   -ks/ms,  -cs/ms,          ks/ms,          cs/ms;
    0,       0,              0,             1;
    ks/mu,   cs/mu, -(ks+kt)/mu, -(cs+ct)/mu
];
B_u = [0; 1/ms; 0; -1/mu];
B_r = [0, 0; 0, 0; 0, 0; kt/mu, ct/mu];

% Precomputed continuous-time LQR gain for
% Q=diag([1000, 50, 500, 20]) and R=0.1.
K = [5.53851381, 16.16044593, -63.16123187, -1.48613719];

fprintf('Passive poles:\n'); disp(eig(A).');
fprintf('Active closed-loop poles:\n'); disp(eig(A - B_u*K).');

dt = 0.0005;
t = 0:dt:3;
road = zeros(size(t));
road_velocity = zeros(size(t));
bump = t >= 0.5 & t <= 0.7;
phase = (t(bump)-0.5)/0.2*pi;
road(bump) = 0.02*sin(phase);
road_velocity(bump) = 0.02*pi/0.2*cos(phase);

x_passive = zeros(4, numel(t));
x_active = zeros(4, numel(t));
u_active = zeros(size(t));
acc_passive = zeros(size(t));
acc_active = zeros(size(t));
u_limit = 50;

for k = 1:numel(t)-1
    disturbance = [road(k); road_velocity(k)];

    dx_passive = A*x_passive(:,k) + B_r*disturbance;
    x_passive(:,k+1) = x_passive(:,k) + dt*dx_passive;

    u_unsaturated = -K*x_active(:,k);
    u_active(k) = min(max(u_unsaturated, -u_limit), u_limit);
    dx_active = A*x_active(:,k) + B_u*u_active(k) + B_r*disturbance;
    x_active(:,k+1) = x_active(:,k) + dt*dx_active;

    acc_passive(k) = dx_passive(2);
    acc_active(k) = dx_active(2);
end
u_active(end) = u_active(end-1);
acc_passive(end) = acc_passive(end-1);
acc_active(end) = acc_active(end-1);

fig = figure('Color', 'w', 'Position', [100, 100, 1100, 780]);
subplot(2,2,1);
plot(t, 1000*road, 'k--', 'LineWidth', 1.2); hold on;
plot(t, 1000*x_passive(1,:), 'LineWidth', 1.4);
plot(t, 1000*x_active(1,:), 'LineWidth', 1.4); grid on;
title('Body displacement'); xlabel('Time [s]'); ylabel('Displacement [mm]');
legend('Road','Passive','Active','Location','best');

subplot(2,2,2);
plot(t, acc_passive, 'LineWidth', 1.4); hold on;
plot(t, acc_active, 'LineWidth', 1.4); grid on;
title('Body acceleration'); xlabel('Time [s]'); ylabel('Acceleration [m/s^2]');
legend('Passive','Active','Location','best');

subplot(2,2,3);
plot(t, 1000*(x_passive(1,:)-x_passive(3,:)), 'LineWidth', 1.4); hold on;
plot(t, 1000*(x_active(1,:)-x_active(3,:)), 'LineWidth', 1.4); grid on;
title('Suspension travel'); xlabel('Time [s]'); ylabel('Travel [mm]');
legend('Passive','Active','Location','best');

subplot(2,2,4);
plot(t, u_active, 'LineWidth', 1.4); grid on;
title('Active suspension force'); xlabel('Time [s]'); ylabel('Force [N]');

sgtitle('Quarter-car active suspension comparison');
out_path = fullfile(script_dir, 'figures', 'active_suspension_results.png');
exportgraphics(fig, out_path, 'Resolution', 160);
fprintf('Peak passive body acceleration: %.3f m/s^2\n', max(abs(acc_passive)));
fprintf('Peak active body acceleration: %.3f m/s^2\n', max(abs(acc_active)));
fprintf('Saved figure: %s\n', out_path);
