function report = ValidateSolution()
% Check the discrete equations and triangle-area constraints in NLP.mod.
global params_
r = params_.kbs; dt = r.terminal_time/(numel(r.x)-1);
residuals = [diff(r.x)-dt*r.v(1:end-1).*cos(r.theta(1:end-1)); ...
    diff(r.y)-dt*r.v(1:end-1).*sin(r.theta(1:end-1)); ...
    diff(r.theta)-dt*r.v(1:end-1).*tan(r.phy(1:end-1))/params_.vehicle.lw; ...
    diff(r.v)-dt*r.a(1:end-1); diff(r.phy)-dt*r.w(1:end-1)];
report.max_dynamics_residual = max(abs(residuals(:)));
boundary = [r.x(1)-params_.task.x0,r.y(1)-params_.task.y0,r.theta(1)-params_.task.theta0, ...
    r.x(end)-params_.task.xf,r.y(end)-params_.task.yf,r.theta(end)-params_.task.thetaf_unwrapped, ...
    r.v([1,end]),r.phy([1,end]),r.a(1),r.w(1)];
report.max_boundary_residual = max(abs(boundary));
report.max_bound_violation = max([0,abs(r.a)-params_.vehicle.amax,abs(r.w)-params_.vehicle.wmax, ...
    abs(r.phy)-params_.vehicle.phymax,r.v-params_.vehicle.vmax,params_.vehicle.vmin-r.v]);
% Evaluate exactly the triangle-area inequalities at the collocation points.
c = cos(r.theta(:)); s = sin(r.theta(:));
front = params_.vehicle.lw+params_.vehicle.lf;
rear = params_.vehicle.lr; half_width = params_.vehicle.lb/2;
vx = r.x(:)+[front*c-half_width*s,front*c+half_width*s,-rear*c+half_width*s,-rear*c-half_width*s];
vy = r.y(:)+[front*s+half_width*c,front*s-half_width*c,-rear*s-half_width*c,-rear*s+half_width*c];
next = [2,3,4,1];
slack = inf;
for ii = 1 : params_.obstacle.num_obs
    obs = params_.obstacle.obs{ii}; ox = obs.x(1:4); oy = obs.y(1:4);
    obstacle_area = hypot(ox(1)-ox(2),oy(1)-oy(2))*hypot(ox(1)-ox(4),oy(1)-oy(4));
    for jj = 1 : 4
        area = 0.5*sum(abs((vx-ox(jj)).*(vy(:,next)-oy(jj))-(vy-oy(jj)).*(vx(:,next)-ox(jj))),2);
        slack = min(slack,min(area-params_.vehicle.length*params_.vehicle.lb-0.01));
        area = 0.5*sum(abs((ox-vx(:,jj)).*(oy(next)-vy(:,jj))-(oy-vy(:,jj)).*(ox(next)-vx(:,jj))),2);
        slack = min(slack,min(area-obstacle_area-0.01));
    end
end
report.min_triangle_area_slack = slack;
report.collision_free = slack >= -params_.opti.validation_tolerance;
report.collision_samples = numel(r.x);
report.passed = report.max_dynamics_residual <= params_.opti.validation_tolerance && ...
    report.max_boundary_residual <= params_.opti.validation_tolerance && ...
    report.max_bound_violation <= params_.opti.validation_tolerance && report.collision_free;
if ~report.passed
    error('KBS2015:ValidationFailed', ...
        'Solution check failed: dynamics %.3g, boundary %.3g, bounds %.3g, collision-free %d.', ...
        report.max_dynamics_residual,report.max_boundary_residual,report.max_bound_violation,report.collision_free);
end
end
