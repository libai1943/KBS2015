function [x,y,theta,v,a,phy,w,terminal_time] = LoadAmplSolution()
global params_
status_file = fullfile('AmplResults','solve_result_num.txt');
if ~isfile(status_file)
    error('KBS2015:MissingStatus','No solver status was written. See AmplResults/solver.log.');
end
status = load(status_file);
if ~isscalar(status) || ~isfinite(status) || status < 0 || status >= 100
    error('KBS2015:SolverFailed','AMPL did not report a solved problem (status %s). See AmplResults/solver.log.',mat2str(status));
end
names = {'x','y','theta','v','a','phy','w','terminal_time'};
values = cell(1,numel(names));
for ii = 1 : numel(names)
    path = fullfile('AmplResults',[names{ii},'.txt']);
    if ~isfile(path), error('KBS2015:MissingOutput','Missing solver output: %s',path); end
    values{ii} = load(path);
    expected = params_.opti.nfe;
    if ii == numel(names), expected = 1; end
    if numel(values{ii}) ~= expected || any(~isfinite(values{ii}(:)))
        error('KBS2015:InvalidOutput','Invalid solver output: %s',path);
    end
    values{ii} = values{ii}(:)';
end
[x,y,theta,v,a,phy,w,terminal_time] = values{:};
if terminal_time <= 0, error('KBS2015:InvalidTime','The solved duration must be positive.'); end
end
