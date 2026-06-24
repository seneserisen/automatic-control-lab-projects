function value = rms_value(signal)
%RMS_VALUE Return the root-mean-square value without toolbox dependencies.

arguments
    signal (:,:) double {mustBeNonempty, mustBeFinite}
end

value = sqrt(mean(signal(:).^2));
end
