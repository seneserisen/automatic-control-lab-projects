function limited = saturate(value, lower_limit, upper_limit)
%SATURATE Clamp a scalar or array to inclusive bounds.

arguments
    value double {mustBeFinite}
    lower_limit (1,1) double {mustBeFinite}
    upper_limit (1,1) double {mustBeFinite}
end

if lower_limit > upper_limit
    error('control_lab:InvalidLimits', 'lower_limit must not exceed upper_limit.');
end
limited = min(max(value, lower_limit), upper_limit);
end
