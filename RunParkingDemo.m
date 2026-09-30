function result = RunParkingDemo(case_id)
% Run HA* initialization, AMPL/IPOPT optimization, and two static figures.
validateattributes(case_id, {'numeric'}, {'scalar', 'integer', '>=', 1, '<=', 4});
root_dir = fileparts(mfilename('fullpath'));
previous_dir = pwd;
restore_dir = onCleanup(@() cd(previous_dir));
cd(root_dir);
if ~ispc
    error('KBS2015:Platform', 'The supplied AMPL/IPOPT executables require Windows.');
end
required_files = {'AMPL.exe', 'ipopt.exe', 'NLP.mod', 'SolveParking.run'};
for ii = 1 : numel(required_files)
    if ~isfile(required_files{ii})
        error('KBS2015:MissingFile', 'Missing required file: %s', required_files{ii});
    end
end
global params_
params_ = struct();
params_.user.case_id = case_id;
InitializeParams();
fprintf('\nKBS 2015 / Chapter 2 -- case %d\n', case_id);
timer = tic;
fprintf('1/3  Searching for a Hybrid A* initial path...\n');
SearchInitialTrajectory();
params_.timing.search = toc(timer);
fprintf('     HA*: %.2f m, initial duration %.3f s (%.2f s computation).\n', ...
    sum(hypot(diff(params_.ha_result.x), diff(params_.ha_result.y))), ...
    params_.ha_result.terminal_time, params_.timing.search);
fprintf('2/3  Solving the triangle-area NLP with AMPL/IPOPT...\n');
timer = tic;
SolveNLP();
params_.timing.optimization = toc(timer);
params_.validation = ValidateSolution();
fprintf('3/3  Drawing the trajectory/footprints and state/control profiles.\n');
DrawResults();
result = params_.kbs;
result.case_id = case_id;
result.initial_guess = params_.ha_result;
result.validation = params_.validation;
result.timing = params_.timing;
fprintf('     Solved: T = %.6f s; maximum Euler residual = %.3g.\n', ...
    result.terminal_time, result.validation.max_dynamics_residual);
fprintf('     Numerical output and solver log: %s\n\n', fullfile(root_dir, 'AmplResults'));
end
