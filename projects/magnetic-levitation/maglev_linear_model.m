function model = maglev_linear_model(configuration)
%MAGLEV_LINEAR_MODEL Build the local deviation model and equilibrium values.

arguments
    configuration (1,1) struct
end

plant = configuration.plant;
equilibrium_current = sqrt( ...
    2*plant.mass*plant.gravity*plant.gap^2/plant.magnetic_coefficient);
equilibrium_voltage = plant.resistance*equilibrium_current;
position_gradient = 2*plant.gravity/plant.gap;
current_gradient = -plant.magnetic_coefficient*equilibrium_current/( ...
    plant.mass*plant.gap^2);

model.A = [
    0, 1, 0;
    position_gradient, 0, current_gradient;
    0, 0, -plant.resistance/plant.inductance
];
model.B = [0; 0; 1/plant.inductance];
model.C = [1, 0, 0; 0, 0, 1];
model.equilibrium_state = [plant.gap; 0; equilibrium_current];
model.equilibrium_current = equilibrium_current;
model.equilibrium_voltage = equilibrium_voltage;
model.open_loop_poles = eig(model.A);
model.observability_rank = rank([model.C; model.C*model.A; model.C*model.A^2]);
end
