%TWO_TANK_DEMO PI saturation and back-calculation anti-windup.

clear; close all; clc;
script_dir = fileparts(mfilename('fullpath'));
if isempty(script_dir), script_dir = pwd; end
repo_root = fileparts(fileparts(script_dir));
addpath(fullfile(repo_root, 'matlab'));

parameters.A1 = 50.0; parameters.A2 = 50.0;
parameters.a12 = 0.20; parameters.a2 = 0.18;
parameters.gravity = 981.0;
controller.Kp = 5.0; controller.Ki = 0.05; controller.Kaw = 0.30;
controller.minimum = 0.0; controller.maximum = 45.0;

initial_h2 = 10.0;
steady_flow = parameters.a2*sqrt(2*parameters.gravity*initial_h2);
initial_h1 = initial_h2+(steady_flow/parameters.a12)^2/(2*parameters.gravity);
initial_integral = steady_flow/controller.Ki;
maximum_level = (controller.maximum/parameters.a2)^2/(2*parameters.gravity);
fprintf('Initial equilibrium: h1=%.4f cm, h2=%.4f cm\n', initial_h1, initial_h2);
fprintf('Pump-limited maximum steady-state h2: %.4f cm\n', maximum_level);

dt = 0.05;
time = 0:dt:700;
reference = initial_h2*ones(size(time));
reference(time >= 100 & time < 160) = 35.0;  % above pump capability
disturbance = zeros(size(time));
disturbance(time >= 520 & time < 580) = -5.0;

without_aw = simulate_two_tank(time, reference, disturbance, parameters, controller, ...
    initial_h1, initial_h2, initial_integral, false);
with_aw = simulate_two_tank(time, reference, disturbance, parameters, controller, ...
    initial_h1, initial_h2, initial_integral, true);

without_recovery = control_lab.sustained_entry_time(time, without_aw.h2-initial_h2, ...
    1.0, 160, 20, 480);
with_recovery = control_lab.sustained_entry_time(time, with_aw.h2-initial_h2, ...
    1.0, 160, 20, 480);

fig = figure('Color', 'w', 'Position', [100, 100, 1100, 780]);
subplot(2,2,1);
plot(time, reference, 'k--', 'LineWidth', 1.3); hold on;
plot(time, without_aw.h2, 'LineWidth', 1.4);
plot(time, with_aw.h2, 'LineWidth', 1.4); grid on;
title('Tank-2 level'); xlabel('Time [s]'); ylabel('Level [cm]');
legend('Reference','No anti-windup','Back-calculation','Location','best');

subplot(2,2,2);
plot(time, without_aw.control, 'LineWidth', 1.4); hold on;
plot(time, with_aw.control, 'LineWidth', 1.4); yline(controller.maximum, ':k'); grid on;
title('Saturated pump command'); xlabel('Time [s]'); ylabel('Flow [cm^3/s]');
legend('No anti-windup','Back-calculation','Limit','Location','best');

subplot(2,2,3);
plot(time, without_aw.integral, 'LineWidth', 1.4); hold on;
plot(time, with_aw.integral, 'LineWidth', 1.4); grid on;
title('Integrator state'); xlabel('Time [s]'); ylabel('Integral state');
legend('No anti-windup','Back-calculation','Location','best');

subplot(2,2,4);
plot(time, disturbance, 'LineWidth', 1.4); grid on;
title('Inlet disturbance'); xlabel('Time [s]'); ylabel('Flow [cm^3/s]');

sgtitle('Two-tank PI control and anti-windup');
out_path = fullfile(script_dir, 'figures', 'two_tank_results.png');
exportgraphics(fig, out_path, 'Resolution', 160);
fprintf('Recovery without anti-windup: %.2f s\n', without_recovery);
fprintf('Recovery with back-calculation: %.2f s\n', with_recovery);
fprintf('Saved figure: %s\n', out_path);

function result = simulate_two_tank(time, reference, disturbance, p, c, ...
        initial_h1, initial_h2, initial_integral, use_anti_windup)
    dt = time(2)-time(1);
    state = zeros(2, numel(time));
    state(:,1) = [initial_h1; initial_h2];
    control = zeros(size(time));
    integral = zeros(size(time));
    integral(1) = initial_integral;
    for idx = 1:numel(time)-1
        error_value = reference(idx)-state(2,idx);
        unsaturated = c.Kp*error_value+c.Ki*integral(idx);
        control(idx) = control_lab.saturate(unsaturated, c.minimum, c.maximum);
        correction = 0;
        if use_anti_windup
            correction = c.Kaw*(control(idx)-unsaturated);
        end
        integral(idx+1) = integral(idx)+dt*(error_value+correction);
        inlet = control(idx)+disturbance(idx);
        rhs = @(~, current_state) tank_dynamics(current_state, inlet, p);
        state(:,idx+1) = max(control_lab.rk4_step(rhs, time(idx), state(:,idx), dt), 0);
    end
    control(end) = control(end-1);
    result.h1 = state(1,:);
    result.h2 = state(2,:);
    result.control = control;
    result.integral = integral;
end

function derivative = tank_dynamics(state, inlet, p)
    h1 = max(state(1), 0); h2 = max(state(2), 0);
    flow_12 = p.a12*sqrt(2*p.gravity*max(h1-h2, 0));
    flow_out = p.a2*sqrt(2*p.gravity*h2);
    derivative = [(inlet-flow_12)/p.A1; (flow_12-flow_out)/p.A2];
end
