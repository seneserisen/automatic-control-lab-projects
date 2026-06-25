function configuration = maglev_configuration()
%MAGLEV_CONFIGURATION Return physical, controller, and simulation settings.

configuration.plant.mass = 0.068;
configuration.plant.gravity = 9.81;
configuration.plant.gap = 0.014;
configuration.plant.magnetic_coefficient = 6.53e-5;
configuration.plant.resistance = 11.0;
configuration.plant.inductance = 0.4125;

configuration.controller.desired_poles = [-20, -30, -40];
configuration.controller.fallback_gain = ...
    [-6316.60910998, -168.35876027, 26.125];
configuration.controller.fallback_prefilter = -1009.79192166;

configuration.observer.desired_poles = [-80, -90, -100];
configuration.observer.fallback_gain = [
    180.000004, 0.0927619903;
    9401.42904, -1.10085054;
    0.00114506735, 63.3333293
];

configuration.measurement.seed = 42;
configuration.measurement.gap_noise_std = 5e-6;
configuration.measurement.current_noise_std = 2e-3;

configuration.simulation.dt = 1e-4;
configuration.simulation.final_time = 0.5;
configuration.reference.start_time = 0.05;
configuration.reference.amplitude = 5e-4;
configuration.voltage.minimum = 0.0;
configuration.voltage.maximum = 30.0;
end
