%RUN_ALL Execute all clean-room MATLAB demonstrations.
% Run this script from the repository root.

project_scripts = {
    'projects/nonlinear-control-loops/nonlinear_control_demo.m'
    'projects/rotary-arm/rotary_arm_demo.m'
    'projects/active-suspension/active_suspension_demo.m'
    'projects/magnetic-levitation/magnetic_levitation_demo.m'
    'projects/two-tank-system/two_tank_demo.m'
};

for idx = 1:numel(project_scripts)
    fprintf('\n=== Running %s ===\n', project_scripts{idx});
    run(project_scripts{idx});
end

fprintf('\nAll demonstrations completed.\n');
