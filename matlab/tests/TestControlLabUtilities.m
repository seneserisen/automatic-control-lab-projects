classdef TestControlLabUtilities < matlab.unittest.TestCase
    methods (Test)
        function rk4MatchesExponentialDecay(testCase)
            state = 1;
            step_size = 0.05;
            for idx = 1:20
                state = control_lab.rk4_step( ...
                    @(~, value) -value, (idx-1)*step_size, state, step_size);
            end
            testCase.verifyEqual(state, exp(-1), 'RelTol', 2e-6);
        end

        function saturationClampsValues(testCase)
            testCase.verifyEqual(control_lab.saturate(12, -5, 5), 5);
            testCase.verifyEqual(control_lab.saturate(-9, -5, 5), -5);
            testCase.verifyEqual(control_lab.saturate(2, -5, 5), 2);
        end

        function saturationRejectsInvalidLimits(testCase)
            testCase.verifyError( ...
                @() control_lab.saturate(0, 1, -1), ...
                'control_lab:InvalidLimits');
        end

        function trackingMetricsAreConsistent(testCase)
            time = 0:0.1:1;
            reference = ones(size(time));
            response = 0.8*ones(size(time));
            control = 2*ones(size(time));
            metrics = control_lab.tracking_metrics( ...
                time, reference, response, control);

            testCase.verifyEqual(metrics.rmse, 0.2, 'AbsTol', 1e-12);
            testCase.verifyEqual(metrics.maximum_absolute_error, 0.2, ...
                'AbsTol', 1e-12);
            testCase.verifyEqual(metrics.iae, 0.2, 'AbsTol', 1e-12);
            testCase.verifyEqual(metrics.ise, 0.04, 'AbsTol', 1e-12);
            testCase.verifyEqual(metrics.steady_state_error, 0.2, ...
                'AbsTol', 1e-12);
            testCase.verifyEqual(metrics.control_rms, 2, 'AbsTol', 1e-12);
            testCase.verifyEqual(metrics.peak_control, 2, 'AbsTol', 1e-12);
        end

        function recoveryTimeRequiresSustainedEntry(testCase)
            time = 0:0.1:5;
            error_signal = ones(size(time));
            error_signal(time >= 1 & time < 1.5) = 0.05;
            error_signal(time >= 3) = 0.05;
            entry_time = control_lab.sustained_entry_time( ...
                time, error_signal, 0.1, 0, 1.0);
            testCase.verifyEqual(entry_time, 3.0, 'AbsTol', 1e-12);
        end
    end
end
