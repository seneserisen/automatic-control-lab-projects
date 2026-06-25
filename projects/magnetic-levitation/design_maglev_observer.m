function design = design_maglev_observer(configuration, model)
%DESIGN_MAGLEV_OBSERVER Design state-feedback and observer gains.

arguments
    configuration (1,1) struct
    model (1,1) struct
end

if exist('place', 'file') == 2
    controller_gain = place(model.A, model.B, ...
        configuration.controller.desired_poles);
    observer_gain = place(model.A', model.C', ...
        configuration.observer.desired_poles)';
    gain_source = 'computed with place';
else
    controller_gain = configuration.controller.fallback_gain;
    observer_gain = configuration.observer.fallback_gain;
    gain_source = 'precomputed fallback';
end

closed_loop = model.A-model.B*controller_gain;
position_output = model.C(1,:);
prefilter_denominator = position_output*(closed_loop\model.B);
if abs(prefilter_denominator) < eps
    prefilter = configuration.controller.fallback_prefilter;
else
    prefilter = -1/prefilter_denominator;
end

observer_matrix = model.A-observer_gain*model.C;
design.controller_gain = controller_gain;
design.observer_gain = observer_gain;
design.prefilter = prefilter;
design.gain_source = gain_source;
design.controller_poles = eig(closed_loop);
design.observer_poles = eig(observer_matrix);
end
