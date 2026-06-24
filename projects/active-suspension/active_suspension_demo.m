%ACTIVE_SUSPENSION_DEMO Quarter-car state-space simulation with LQR feedback.

clear; close all; clc;
script_dir = fileparts(mfilename('fullpath'));
if isempty(script_dir), script_dir = pwd; end
repo_root = fileparts(fileparts(script_dir));
addpath(fullfile(repo_root, 'matlab'));

parameters.ms = 2.45;  parameters.mu = 1.40;
parameters.ks = 900;   parameters.kt = 2500;
parameters.cs = 12;    parameters.ct = 10;

A = [
    0, 1, 0, 0;
   -parameters.ks/parameters.ms, -parameters.cs/parameters.ms, parameters.ks/parameters.ms, parameters.cs/parameters.ms;
    0, 0, 0, 1;
    parameters.ks/parameters.mu, parameters.cs/parameters.mu, ...
    -(parameters.ks+parameters.kt)/parameters.mu, -(parameters.cs+parameters.ct)/parameters.mu
];
B_control = [0; 1/parameters.ms; 0; -1/parameters.mu];
B_road = [0,0; 0,0; 0,0; parameters.kt/parameters.mu, parameters.ct/parameters.mu];
Q = diag([1000, 50, 500, 20]);
R = 0.1;

if exist('lqr', 'file') == 2
    gain = lqr(A, B_control, Q, R);
    gain_source = 'computed with lqr';
else
    gain = [5.53851381, 16.16044593, -63.16123187, -1.48613719];
    gain_source = 'precomputed fallback';
end
fprintf('LQR gain source: %s\n', gain_source);
fprintf('Passive poles:\n'); disp(eig(A).');
fprintf('Active closed-loop poles:\n'); disp(eig(A-B_control*gain).');

dt = 0.0005;
time = 0:dt:3;
road = zeros(size(time));
road_velocity = zeros(size(time));
bump = time >= 0.5 & time <= 0.7;
phase = (time(bump)-0.5)/0.2*pi;
road(bump) = 0.02*sin(phase);
road_velocity(bump) = 0.02*pi/0.2*cos(phase);

passive = zeros(4, numel(time));
active = zeros(4, numel(time));
passive_acceleration = zeros(size(time));
active_acceleration = zeros(size(time));
active_force = zeros(size(time));
force_limit = 50;

for idx = 1:numel(time)-1
    disturbance = [road(idx); road_velocity(idx)];
    force = control_lab.saturate(-gain*active(:,idx), -force_limit, force_limit);
    active_force(idx) = force;
    passive_rhs = @(~, state) A*state+B_road*disturbance;
    active_rhs = @(~, state) A*state+B_control*force+B_road*disturbance;
    passive(:,idx+1) = control_lab.rk4_step(passive_rhs, time(idx), passive(:,idx), dt);
    active(:,idx+1) = control_lab.rk4_step(active_rhs, time(idx), active(:,idx), dt);
    passive_derivative = passive_rhs(time(idx), passive(:,idx));
    active_derivative = active_rhs(time(idx), active(:,idx));
    passive_acceleration(idx) = passive_derivative(2);
    active_acceleration(idx) = active_derivative(2);
end
active_force(end) = active_force(end-1);
passive_acceleration(end) = passive_acceleration(end-1);
active_acceleration(end) = active_acceleration(end-1);

passive_travel = passive(1,:)-passive(3,:);
active_travel = active(1,:)-active(3,:);
passive_tire_deflection = passive(3,:)-road;
active_tire_deflection = active(3,:)-road;

fig = figure('Color', 'w', 'Position', [100, 100, 1100, 780]);
subplot(2,2,1);
plot(time, passive_acceleration, 'LineWidth', 1.4); hold on;
plot(time, active_acceleration, 'LineWidth', 1.4); grid on;
title('Body acceleration'); xlabel('Time [s]'); ylabel('m/s^2');
legend('Passive','Active','Location','best');

subplot(2,2,2);
plot(time, 1000*passive_travel, 'LineWidth', 1.4); hold on;
plot(time, 1000*active_travel, 'LineWidth', 1.4); grid on;
title('Suspension travel'); xlabel('Time [s]'); ylabel('mm');
legend('Passive','Active','Location','best');

subplot(2,2,3);
plot(time, 1000*passive_tire_deflection, 'LineWidth', 1.4); hold on;
plot(time, 1000*active_tire_deflection, 'LineWidth', 1.4); grid on;
title('Tire deflection'); xlabel('Time [s]'); ylabel('mm');
legend('Passive','Active','Location','best');

subplot(2,2,4);
plot(time, active_force, 'LineWidth', 1.4); grid on;
title('Active force'); xlabel('Time [s]'); ylabel('N');

sgtitle('Quarter-car active suspension comparison');
out_path = fullfile(script_dir, 'figures', 'active_suspension_results.png');
exportgraphics(fig, out_path, 'Resolution', 160);
fprintf('Passive acceleration RMS: %.4f m/s^2\n', control_lab.rms_value(passive_acceleration));
fprintf('Active acceleration RMS: %.4f m/s^2\n', control_lab.rms_value(active_acceleration));
fprintf('Peak active force: %.3f N\n', max(abs(active_force)));
fprintf('Saved figure: %s\n', out_path);
