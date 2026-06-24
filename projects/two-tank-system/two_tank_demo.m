%TWO_TANK_DEMO Nonlinear hydraulic process with PI anti-windup comparison.

clear; close all; clc;
script_dir = fileparts(mfilename('fullpath'));
if isempty(script_dir), script_dir = pwd; end

params.A1 = 50.0;     % tank area [cm^2]
params.A2 = 50.0;
params.a12 = 0.20;    % inter-tank outlet area [cm^2]
params.a2 = 0.18;     % final outlet area [cm^2]
params.g = 981.0;     % gravity [cm/s^2]

controller.Kp = 5.0;
controller.Ki = 0.05;
controller.Kaw = 0.30;
controller.u_min = 0.0;
controller.u_max = 45.0;

dt = 0.05;
t = 0:dt:700;
reference = 10*ones(size(t));
reference(t >= 100 & t < 300) = 25;  % deliberately unreachable demand

disturbance = zeros(size(t));
disturbance(t >= 450 & t < 520) = -5.0;

[result_no_aw] = simulate_two_tank(t, reference, disturbance, params, controller, false);
[result_aw] = simulate_two_tank(t, reference, disturbance, params, controller, true);

fig = figure('Color', 'w', 'Position', [100, 100, 1100, 760]);
subplot(2,2,1);
plot(t, reference, 'k--', 'LineWidth', 1.3); hold on;
plot(t, result_no_aw.h2, 'LineWidth', 1.4);
plot(t, result_aw.h2, 'LineWidth', 1.4); grid on;
title('Tank-2 level'); xlabel('Time [s]'); ylabel('Level [cm]');
legend('Reference','PI without anti-windup','PI with anti-windup','Location','best');

subplot(2,2,2);
plot(t, result_no_aw.u_sat, 'LineWidth', 1.4); hold on;
plot(t, result_aw.u_sat, 'LineWidth', 1.4); yline(controller.u_max, ':k'); grid on;
title('Saturated pump command'); xlabel('Time [s]'); ylabel('Flow [cm^3/s]');
legend('No anti-windup','Back-calculation','Upper limit','Location','best');

subplot(2,2,3);
plot(t, result_no_aw.integral, 'LineWidth', 1.4); hold on;
plot(t, result_aw.integral, 'LineWidth', 1.4); grid on;
title('Integrator state'); xlabel('Time [s]'); ylabel('Integral state');
legend('No anti-windup','Back-calculation','Location','best');

subplot(2,2,4);
plot(t, disturbance, 'LineWidth', 1.4); grid on;
title('Inlet disturbance'); xlabel('Time [s]'); ylabel('Flow [cm^3/s]');

sgtitle('Two-tank PI control and anti-windup');
out_path = fullfile(script_dir, 'figures', 'two_tank_results.png');
exportgraphics(fig, out_path, 'Resolution', 160);
fprintf('Saved figure: %s\n', out_path);

function result = simulate_two_tank(t, reference, disturbance, p, c, anti_windup)
    dt = t(2)-t(1);
    n = numel(t);
    h1 = zeros(1,n);
    h2 = zeros(1,n);
    u_sat = zeros(1,n);
    integral = zeros(1,n);

    for k = 1:n-1
        error = reference(k) - h2(k);
        u_unsat = c.Kp*error + c.Ki*integral(k);
        u_sat(k) = min(max(u_unsat, c.u_min), c.u_max);

        correction = 0;
        if anti_windup
            correction = c.Kaw*(u_sat(k)-u_unsat);
        end
        integral(k+1) = integral(k) + dt*(error + correction);

        q12 = p.a12*sqrt(2*p.g*max(h1(k)-h2(k), 0));
        qout = p.a2*sqrt(2*p.g*max(h2(k), 0));
        dh1 = (u_sat(k) + disturbance(k) - q12)/p.A1;
        dh2 = (q12 - qout)/p.A2;

        h1(k+1) = max(h1(k) + dt*dh1, 0);
        h2(k+1) = max(h2(k) + dt*dh2, 0);
    end
    u_sat(end) = u_sat(end-1);

    result.h1 = h1;
    result.h2 = h2;
    result.u_sat = u_sat;
    result.integral = integral;
end
