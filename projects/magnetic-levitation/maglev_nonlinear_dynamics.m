function derivative = maglev_nonlinear_dynamics(state, voltage, configuration)
%MAGLEV_NONLINEAR_DYNAMICS Evaluate the nonlinear magnetic-levitation plant.

arguments
    state (3,1) double {mustBeFinite}
    voltage (1,1) double {mustBeFinite}
    configuration (1,1) struct
end

plant = configuration.plant;
gap = max(state(1), 0.004);
current = max(state(3), 0.0);
magnetic_force = plant.magnetic_coefficient*current^2/(2*gap^2);

derivative = [
    state(2);
    plant.gravity-magnetic_force/plant.mass;
    (voltage-plant.resistance*state(3))/plant.inductance
];
end
