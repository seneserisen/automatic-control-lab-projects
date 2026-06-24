function next_state = rk4_step(rhs, time, state, step_size)
%RK4_STEP Advance one classical fourth-order Runge-Kutta step.

arguments
    rhs (1,1) function_handle
    time (1,1) double
    state (:,1) double
    step_size (1,1) double {mustBePositive, mustBeFinite}
end

k1 = rhs(time, state);
k2 = rhs(time + 0.5*step_size, state + 0.5*step_size*k1);
k3 = rhs(time + 0.5*step_size, state + 0.5*step_size*k2);
k4 = rhs(time + step_size, state + step_size*k3);
next_state = state + step_size*(k1 + 2*k2 + 2*k3 + k4)/6;
end
