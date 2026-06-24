%ROTARY_ARM_DEMO Compare feedback-only and two-degree-of-freedom arm control.

clear; close all; clc;
script_dir = fileparts(mfilename('fullpath'));
if isempty(script_dir), script_dir = pwd; end
repo_root = fileparts(fileparts(script_dir));
addpath(fullfile(repo_root, 'matlab'));

% Simplified third-order rotary-arm model:
% y''' + a2*y'' + a1*y' = b*u
model.a1 = 6.0;
model.a2 = 2.0;
model.b = 1.0;

transition_time = 3.0;
final_time = 6.0;
dt = 0.002;
time = 0:dt:final_time;
[reference, velocity_ref, acceleration_ref, jerk_ref] = fifth_order_trajectory(...
    time, transition_time, 0.0, 1.0);
feedforward = (jerk_ref + model.a2*acceleration_ref + model.a1*velocity_ref)/model.b;

% Feedback gains produce the error polynomial (s+4)(s+5)(s+6).
gain = [120, 68, 13];
control_limit = 25;
disturbance = zeros(size(time));
disturbance(time >= 3.6 & time < 3.9) = -4.0;

feedback_only = simulate_arm(time, reference, velocity_ref, acceleration_ref, ...
    zeros(size(feedforward)), disturbance, model, gain, control_limit);
two_dof = simulate_arm(time, reference, velocity_ref, acceleration_ref, ...
    feedforward, disturbance, model, gain, control_limit);

feedback_rmse = control_lab.rms_value(reference - feedback_only.position);
two_dof_rmse = control_lab.rms_value(reference - two_dof.position);

fig = figure('Color', 'w', 'Position', [100, 100, 1100, 780]);
subplot(2,2,1);
plot(time, reference, 'k--', 'LineWidth', 1.4); hold on;
plot(time, feedback_only.position, 'LineWidth', 1.4);
plot(time, two_dof.position, 'LineWidth', 1.5); grid on;
title('Reference tracking'); xlabel('Time [s]'); ylabel('Angle [rad]');
legend('Reference','Feedback only','2-DOF','Location','best');

subplot(2,2,2);
plot(time, feedback_only.control, 'LineWidth', 1.3); hold on;
plot(time, two_dof.control, 'LineWidth', 1.4);
yline(control_limit, ':k'); yline(-control_limit, ':k'); grid on;
title('Control effort'); xlabel('Time [s]'); ylabel('Input');
legend('Feedback only','2-DOF','Limits','Location','best');

subplot(2,2,3);
plot(time, disturbance, 'LineWidth', 1.4); grid on;
title('Injected load disturbance'); xlabel('Time [s]'); ylabel('Equivalent input');

subplot(2,2,4);
plot(time, reference-feedback_only.position, 'LineWidth', 1.3); hold on;
plot(time, reference-two_dof.position, 'LineWidth', 1.4); grid on;
title('Tracking error'); xlabel('Time [s]'); ylabel('Error [rad]');
legend('Feedback only','2-DOF','Location','best');

sgtitle('Rotary-arm feedback and two-degree-of-freedom control');
out_path = fullfile(script_dir, 'figures', 'rotary_arm_results.png');
exportgraphics(fig, out_path, 'Resolution', 160);
fprintf('Feedback-only tracking RMSE: %.6f rad\n', feedback_rmse);
fprintf('2-DOF tracking RMSE: %.6f rad\n', two_dof_rmse);
fprintf('Peak 2-DOF control magnitude: %.3f\n', max(abs(two_dof.control)));
fprintf('Saved figure: %s\n', out_path);

function result = simulate_arm(time, reference, velocity_ref, acceleration_ref, ...
        feedforward, disturbance, model, gain, control_limit)
    dt = time(2)-time(1);
    state = zeros(3, numel(time));
    control = zeros(size(time));
    for idx = 1:numel(time)-1
        target = [reference(idx); velocity_ref(idx); acceleration_ref(idx)];
        control(idx) = control_lab.saturate(...
            feedforward(idx) + gain*(target-state(:,idx)), -control_limit, control_limit);
        total_input = control(idx) + disturbance(idx);
        rhs = @(~, current_state) [
            current_state(2);
            current_state(3);
            -model.a1*current_state(2)-model.a2*current_state(3)+model.b*total_input
        ];
        state(:,idx+1) = control_lab.rk4_step(rhs, time(idx), state(:,idx), dt);
    end
    control(end) = control(end-1);
    result.position = state(1,:);
    result.control = control;
end

function [position, velocity, acceleration, jerk] = fifth_order_trajectory(time, T, initial, final)
    tau = min(max(time/T, 0), 1);
    delta = final-initial;
    position = initial + delta*(10*tau.^3-15*tau.^4+6*tau.^5);
    velocity = delta/T*(30*tau.^2-60*tau.^3+30*tau.^4);
    acceleration = delta/T^2*(60*tau-180*tau.^2+120*tau.^3);
    jerk = delta/T^3*(60-360*tau+360*tau.^2);
    finished = time > T;
    velocity(finished) = 0;
    acceleration(finished) = 0;
    jerk(finished) = 0;
end
