function figure_handle = plot_maglev_observer_results(result, metrics, output_path)
%PLOT_MAGLEV_OBSERVER_RESULTS Visualize tracking, estimation, and control effort.

arguments
    result (1,1) struct
    metrics (1,1) struct
    output_path (1,:) char = ''
end

figure_handle = figure('Color', 'w', 'Position', [100, 100, 1150, 820]);

subplot(2,2,1);
plot(result.time, 1000*result.reference, 'k--', 'LineWidth', 1.4); hold on;
plot(result.time, 1000*result.full_state_deviation(1,:), 'LineWidth', 1.3);
plot(result.time, 1000*result.observer_deviation(1,:), 'LineWidth', 1.5);
grid on;
title('Reference tracking'); xlabel('Time [s]'); ylabel('Gap deviation [mm]');
legend('Reference', 'Full-state feedback', 'Observer feedback', 'Location', 'best');

subplot(2,2,2);
plot(result.time, result.observer_deviation(2,:), 'LineWidth', 1.3); hold on;
plot(result.time, result.estimated_state(2,:), '--', 'LineWidth', 1.4);
grid on;
title(sprintf('Velocity estimation: RMSE %.3g m/s', ...
    metrics.velocity_estimation_rmse));
xlabel('Time [s]'); ylabel('Velocity [m/s]');
legend('True', 'Estimated', 'Location', 'best');

subplot(2,2,3);
plot(result.time, result.observer_voltage, 'LineWidth', 1.4); hold on;
yline(30, ':k'); yline(0, ':k');
grid on;
title('Observer-based control voltage'); xlabel('Time [s]'); ylabel('Voltage [V]');

subplot(2,2,4);
plot(result.time, 1000*result.estimation_error(1,:), 'LineWidth', 1.3); hold on;
plot(result.time, 1000*result.gap_noise, ':', 'LineWidth', 1.0);
grid on;
title(sprintf('Position estimate: RMSE %.4g mm', ...
    1000*metrics.position_estimation_rmse));
xlabel('Time [s]'); ylabel('Error [mm]');
legend('Estimation error', 'Measurement noise', 'Location', 'best');

sgtitle('Magnetic levitation with output-feedback observer');
if ~isempty(output_path)
    exportgraphics(figure_handle, output_path, 'Resolution', 160);
end
end
