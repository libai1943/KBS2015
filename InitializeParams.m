function InitializeParams()
global params_
LoadCase();
params_.scenario.xmin = -15;
params_.scenario.xmax = 15;
params_.scenario.ymin = -15;
params_.scenario.ymax = 15;

% Table 2-1 in Chapter 2. Angles are in radians; lengths are in metres.
params_.vehicle.lw = 2.8;
params_.vehicle.lf = 0.96;
params_.vehicle.lr = 0.929;
params_.vehicle.lb = 1.942;
params_.vehicle.length = params_.vehicle.lw + params_.vehicle.lf + params_.vehicle.lr;
params_.vehicle.vmax = 1;
params_.vehicle.vmin = -1;
params_.vehicle.amax = 1;
params_.vehicle.phymax = 0.5;
params_.vehicle.wmax = 0.35;
params_.opti.nfe = 200;
params_.opti.validation_tolerance = 1e-5;

params_.hybrid_astar.map_resolution = 5;
params_.hybrid_astar.simulation_step = max(0.6,0.5*params_.vehicle.length);
params_.hybrid_astar.analytic_expansion_interval = 3;
params_.hybrid_astar.collision_step = 0.08;
params_.hybrid_astar.penalty_for_backward = 0.2;
params_.hybrid_astar.penalty_for_direction_change = 1.5;
if params_.user.case_id == 1
    % The narrow parallel bay requires shorter parking shunts.
    params_.hybrid_astar.map_resolution = 20;
    params_.hybrid_astar.simulation_step = 0.4;
    params_.hybrid_astar.analytic_expansion_interval = 1;
end
params_.utility.ego_vehicle_rgb = [0.45, 0.65, 0.83];

if ~isfolder('AmplInputs'), mkdir('AmplInputs'); end
if ~isfolder('AmplResults'), mkdir('AmplResults'); end
values = [params_.vehicle.lw, params_.vehicle.lf, params_.vehicle.lr, ...
    params_.vehicle.lb, params_.vehicle.vmax, params_.vehicle.vmin, ...
    params_.vehicle.amax, params_.vehicle.phymax, params_.vehicle.wmax, ...
    params_.opti.nfe, params_.obstacle.num_obs];
fid = fopen(fullfile('AmplInputs', 'BasicParameters.txt'), 'w');
if fid < 0, error('KBS2015:FileWrite', 'Cannot write AMPL parameters.'); end
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, '%d %.17g\n', [1:numel(values); values]);
end
