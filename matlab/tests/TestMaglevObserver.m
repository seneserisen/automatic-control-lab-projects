classdef TestMaglevObserver < matlab.unittest.TestCase
    properties
        Configuration
        Model
        Design
    end

    methods (TestMethodSetup)
        function createExperiment(testCase)
            testCase.Configuration = maglev_configuration();
            testCase.Model = maglev_linear_model(testCase.Configuration);
            testCase.Design = design_maglev_observer( ...
                testCase.Configuration, testCase.Model);
        end
    end

    methods (Test)
        function modelIsObservable(testCase)
            testCase.verifyEqual(testCase.Model.observability_rank, 3);
            testCase.verifyEqual(testCase.Design.observability_rank, 3);
        end

        function controllerAndObserverAreStable(testCase)
            testCase.verifyLessThan( ...
                max(real(testCase.Design.controller_poles)), 0);
            testCase.verifyLessThan( ...
                max(real(testCase.Design.observer_poles)), 0);
        end

        function observerTracksWithSensorNoise(testCase)
            result = simulate_maglev_observer( ...
                testCase.Configuration, testCase.Model, testCase.Design);
            metrics = calculate_maglev_observer_metrics( ...
                result, testCase.Design);

            testCase.verifyLessThan( ...
                abs(result.observer_deviation(1,end)-5e-4), 1e-5);
            testCase.verifyLessThan(metrics.position_estimation_rmse, 2e-6);
            testCase.verifyLessThan(metrics.velocity_estimation_rmse, 1e-4);
            testCase.verifyLessThan(metrics.current_estimation_rmse, 5e-4);
            testCase.verifyLessThanOrEqual(metrics.maximum_voltage, 30);
            testCase.verifyGreaterThanOrEqual(metrics.minimum_voltage, 0);
        end

        function fixedSeedIsDeterministic(testCase)
            first = simulate_maglev_observer( ...
                testCase.Configuration, testCase.Model, testCase.Design);
            second = simulate_maglev_observer( ...
                testCase.Configuration, testCase.Model, testCase.Design);

            testCase.verifyEqual(first.observer_deviation, ...
                second.observer_deviation, 'AbsTol', 0);
            testCase.verifyEqual(first.estimated_state, ...
                second.estimated_state, 'AbsTol', 0);
        end

        function fixedStepSolutionConverges(testCase)
            study = maglev_convergence_study([2e-4, 1e-4, 5e-5]);
            testCase.verifyLessThan( ...
                study.final_position_difference_from_finest(end-1), 1e-8);
            testCase.verifyLessThan( ...
                study.voltage_difference_from_finest(end-1), 1e-3);
        end
    end
end
