function WriteBoundaryValues()
global params_
fid = fopen(fullfile('AmplInputs','BoundaryValues.txt'),'w');
if fid < 0, error('KBS2015:FileWrite','Cannot write boundary values.'); end
cleanup = onCleanup(@() fclose(fid));
% Select the equivalent 2*pi branch without changing the requested pose.
thetaf = params_.task.thetaf + 2*pi*round((params_.ha_result.theta(end)-params_.task.thetaf)/(2*pi));
values = [params_.task.x0,params_.task.y0,params_.task.theta0, ...
    params_.task.xf,params_.task.yf,thetaf];
params_.task.thetaf_unwrapped = thetaf;
fprintf(fid,'%d %.17g\n',[1:6;values]);
end
