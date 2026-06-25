function entry_time = sustained_entry_time(time, error_signal, tolerance, start_time, hold_time, end_time)
%SUSTAINED_ENTRY_TIME First time error remains inside a tolerance band.

arguments
    time (1,:) double {mustBeFinite, mustBeNonempty}
    error_signal (1,:) double {mustBeFinite}
    tolerance (1,1) double {mustBePositive, mustBeFinite}
    start_time (1,1) double {mustBeFinite}
    hold_time (1,1) double {mustBePositive, mustBeFinite}
    end_time (1,1) double = inf
end

if numel(time) ~= numel(error_signal)
    error('control_lab:SizeMismatch', ...
        'time and error_signal must have equal lengths.');
end
if numel(time) < 2
    error('control_lab:NotEnoughSamples', ...
        'At least two time samples are required.');
end
if isnan(end_time) || end_time < start_time
    error('control_lab:InvalidEndTime', ...
        'end_time must be greater than or equal to start_time.');
end

dt = median(diff(time));
window = max(1, ceil(hold_time/dt));
inside = time >= start_time & time <= end_time & ...
    abs(error_signal) <= tolerance;
run_length = 0;
entry_time = NaN;
for idx = 1:numel(time)
    if inside(idx)
        run_length = run_length + 1;
    else
        run_length = 0;
    end
    if run_length >= window
        entry_time = time(idx-window+1);
        return;
    end
end
end
