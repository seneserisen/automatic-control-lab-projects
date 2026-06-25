function metrics = tracking_metrics(time, reference, response, control)
%TRACKING_METRICS Calculate common tracking and control-effort metrics.

arguments
    time (1,:) double {mustBeFinite, mustBeNonempty}
    reference (1,:) double {mustBeFinite}
    response (1,:) double {mustBeFinite}
    control (1,:) double {mustBeFinite} = []
end

if numel(time) ~= numel(reference) || numel(time) ~= numel(response)
    error('control_lab:SizeMismatch', ...
        'time, reference, and response must have equal lengths.');
end
if numel(time) < 2 || any(diff(time) <= 0)
    error('control_lab:InvalidTime', ...
        'time must contain at least two strictly increasing samples.');
end
if ~isempty(control) && numel(control) ~= numel(time)
    error('control_lab:SizeMismatch', ...
        'control must be empty or have the same length as time.');
end

tracking_error = reference - response;
metrics.rmse = control_lab.rms_value(tracking_error);
metrics.maximum_absolute_error = max(abs(tracking_error));
metrics.iae = trapz(time, abs(tracking_error));
metrics.ise = trapz(time, tracking_error.^2);
metrics.itae = trapz(time, time.*abs(tracking_error));
metrics.steady_state_error = tracking_error(end);

if isempty(control)
    metrics.control_rms = NaN;
    metrics.peak_control = NaN;
else
    metrics.control_rms = control_lab.rms_value(control);
    metrics.peak_control = max(abs(control));
end
end
