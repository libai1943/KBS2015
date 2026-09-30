function SolveNLP()
global params_
initial = params_.ha_result;
FormInitialGuessViaFullConfig(initial.x,initial.y,initial.theta,initial.v, ...
    initial.a,initial.phy,initial.w,initial.terminal_time);
WriteBoundaryValues();
WriteObs();
% Remove only this solver's previous outputs so a failed run cannot reuse them.
names = {'x','y','theta','v','a','phy','w','terminal_time','solve_result_num','solve_result'};
for ii = 1 : numel(names)
    path = fullfile('AmplResults',[names{ii},'.txt']);
    if isfile(path), delete(path); end
end
% Executables, model and .run live in the current directory (set by the entry).
% Keep solver threads bounded: large default OpenMP pools slow small NLPs down.
old_threads = getenv('OMP_NUM_THREADS');
restore_threads = onCleanup(@() setenv('OMP_NUM_THREADS',old_threads));
setenv('OMP_NUM_THREADS','1');
[exit_code,output] = system('".\AMPL.exe" SolveParking.run 2>&1');
fid = fopen(fullfile('AmplResults','solver.log'),'w');
if fid >= 0, fprintf(fid,'%s',output); fclose(fid); end
if exit_code ~= 0
    error('KBS2015:SolverProcess','AMPL/IPOPT exited with code %d. See AmplResults/solver.log.\n%s',exit_code,output);
end
[params_.kbs.x,params_.kbs.y,params_.kbs.theta,params_.kbs.v,params_.kbs.a, ...
    params_.kbs.phy,params_.kbs.w,params_.kbs.terminal_time] = LoadAmplSolution();
params_.kbs.solve_result_num = load(fullfile('AmplResults','solve_result_num.txt'));
end
