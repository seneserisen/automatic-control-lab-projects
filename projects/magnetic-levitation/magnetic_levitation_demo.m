%MAGNETIC_LEVITATION_DEMO Linearised maglev model and state-feedback stabilisation.

clear; close all; clc;
script_dir = fileparts(mfilename('fullpath'));
if isempty(script_dir), script_dir = pwd; end

m = 0.068;       % ball mass [kg]
g = 9.81;        % gravity [m/s^2]
x0 = 0.014;      % equilibrium air gap [m]
Km = 6.53e-5;    % magnetic-force coefficient
R = 11.0;        % total coil resistance [ohm]
L = 0.4125;      % coil inductance [H]

i0 = sqrt(2*m*g*x0^2/Km);
fprintf('Equilibrium coil current: %.4f A\n', i0);

% Linearised deviation model. The magnetic subsystem matches the verified
% laboratory form approximately: Gm(s) = -9.804 / (s^2 - 1401).
a = 2*g/x0;
b = -9.804;
A = [
    0, 1, 0;
    a, 0, b;
    0, 0, -R/L
];
B = [0; 0; 1/L];
C = [1, 0, 0];

fprintf('Open-loop poles:\n'); disp(eig(A).');

% Pole-placement gain for desired poles [-20, -30, -40].
K = [-6316.60910998, -168.35876027, 26.125];
Nbar = -1009.79192166;
fprintf('Closed-loop poles:\n'); disp(eig(A - B*K).');

dt = 0.0001;
t = 0:dt:0.5;
reference = zeros(size(t));
reference(t >= 0.05) = 0.001;  % 1 mm air-gap deviation

x = zeros(3, numel(t));
u = zeros(size(t));
u_limit = 10;
for k = 1:numel(t)-1
    u_unsaturated = -K*x(:,k) + Nbar*reference(k);
    u(k) = min(max(u_unsaturated, -u_limit), u_limit);
    dx = A*x(:,k) + B*u(k);
    x(:,k+1) = x(:,k) + dt*dx;
end
u(end) = u(end-1);

% Open-loop magnetic frequency response from current deviation to position.
w = logspace(0, 3, 600);
s = 1i*w;
Gm = b ./ (s.^2 - a);

fig = figure('Color', 'w', 'Position', [100, 100, 1100, 760]);
subplot(2,2,1);
plot(t, 1000*reference, '--', 'LineWidth', 1.5); hold on;
plot(t, 1000*x(1,:), 'LineWidth', 1.6); grid on;
title('Air-gap reference tracking'); xlabel('Time [s]'); ylabel('Deviation [mm]');
legend('Reference','Response','Location','best');

subplot(2,2,2);
plot(t, x(3,:), 'LineWidth', 1.5); grid on;
title('Coil-current deviation'); xlabel('Time [s]'); ylabel('Current [A]');

subplot(2,2,3);
semilogx(w, 20*log10(abs(Gm)), 'LineWidth', 1.5); grid on;
title('Magnetic subsystem magnitude'); xlabel('Frequency [rad/s]'); ylabel('Magnitude [dB]');

subplot(2,2,4);
plot(real(Gm), imag(Gm), 'LineWidth', 1.4); hold on;
plot(-1, 0, 'rx', 'MarkerSize', 9, 'LineWidth', 1.5); axis equal; grid on;
title('Open-loop magnetic Nyquist locus'); xlabel('Real'); ylabel('Imaginary');

sgtitle('Magnetic levitation modelling and stabilisation');
out_path = fullfile(script_dir, 'figures', 'magnetic_levitation_results.png');
exportgraphics(fig, out_path, 'Resolution', 160);
fprintf('Saved figure: %s\n', out_path);
