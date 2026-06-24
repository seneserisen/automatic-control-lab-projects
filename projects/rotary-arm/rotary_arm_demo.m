%ROTARY_ARM_DEMO Smooth trajectory generation and model-based feedforward control.

clear; close all; clc;
script_dir = fileparts(mfilename('fullpath'));
if isempty(script_dir), script_dir = pwd; end

% Simplified third-order rotary-arm model:
% y''' + a2*y'' + a1*y' = b*u
model.a1 = 6.0;
model.a2 = 2.0;
model.b = 1.0;

transition_time = 3.0;
final_time = 5.0;
dt = 0.002;
t = 0:dt:final_time;

[r, r_dot, r_ddot, r_dddot] = fifth_order_trajectory(t, transition_time, 0.0, 1.0);
u_ff = (r_dddot + model.a2*r_ddot + model.a1*r_dot) / model.b;

% Feedback gains chosen for error poles at -4, -5 and -6.
K = [120, 68, 13];
u_limit = 25;

x = zeros(3, numel(t));
u = zeros(size(t));
for k = 1:numel(t)-1
    target = [r(k); r_dot(k); r_ddot(k)];
    u_unsaturated = u_ff(k) + K*(target - x(:,k));
    u(k) = min(max(u_unsaturated, -u_limit), u_limit);

    dx = [
        x(2,k);
        x(3,k);
        -model.a1*x(2,k) - model.a2*x(3,k) + model.b*u(k)
    ];
    x(:,k+1) = x(:,k) + dt*dx;
end
u(end) = u(end-1);

% Manual frequency response of G(s)=b/[s(s^2+a2*s+a1)].
w = logspace(-1, 2, 500);
s = 1i*w;
G = model.b ./ (s .* (s.^2 + model.a2*s + model.a1));
mag_db = 20*log10(abs(G));
phase_deg = unwrap(angle(G))*180/pi;

fig = figure('Color', 'w', 'Position', [100, 100, 1100, 760]);
subplot(2,2,1);
plot(t, r, '--', 'LineWidth', 1.5); hold on;
plot(t, x(1,:), 'LineWidth', 1.6); grid on;
title('Reference tracking'); xlabel('Time [s]'); ylabel('Angle [rad]');
legend('Reference','Arm response','Location','best');

subplot(2,2,2);
plot(t, u_ff, '--', 'LineWidth', 1.3); hold on;
plot(t, u, 'LineWidth', 1.5); yline(u_limit, ':k'); yline(-u_limit, ':k'); grid on;
title('Feedforward and total control'); xlabel('Time [s]'); ylabel('Input');
legend('Feedforward','Total control','Actuator limits','Location','best');

subplot(2,2,3);
semilogx(w, mag_db, 'LineWidth', 1.5); grid on;
title('Open-loop magnitude'); xlabel('Frequency [rad/s]'); ylabel('Magnitude [dB]');

subplot(2,2,4);
semilogx(w, phase_deg, 'LineWidth', 1.5); grid on;
title('Open-loop phase'); xlabel('Frequency [rad/s]'); ylabel('Phase [deg]');

sgtitle('Rotary-arm trajectory and feedforward control');
out_path = fullfile(script_dir, 'figures', 'rotary_arm_results.png');
exportgraphics(fig, out_path, 'Resolution', 160);
fprintf('Maximum feedforward input: %.3f\n', max(abs(u_ff)));
fprintf('Maximum tracking error: %.6f rad\n', max(abs(r - x(1,:))));
fprintf('Saved figure: %s\n', out_path);

function [r, r_dot, r_ddot, r_dddot] = fifth_order_trajectory(t, T, r0, rf)
    tau = min(max(t/T, 0), 1);
    delta = rf - r0;

    r = r0 + delta*(10*tau.^3 - 15*tau.^4 + 6*tau.^5);
    r_dot = delta/T*(30*tau.^2 - 60*tau.^3 + 30*tau.^4);
    r_ddot = delta/T^2*(60*tau - 180*tau.^2 + 120*tau.^3);
    r_dddot = delta/T^3*(60 - 360*tau + 360*tau.^2);

    finished = t > T;
    r_dot(finished) = 0;
    r_ddot(finished) = 0;
    r_dddot(finished) = 0;
end
