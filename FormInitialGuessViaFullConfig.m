function FormInitialGuessViaFullConfig(x,y,theta,v,a,phy,w,terminal_time)
global params_
fid = fopen(fullfile('AmplInputs','InitialGuess.INIVAL'),'w');
if fid < 0, error('KBS2015:FileWrite','Cannot write the initial guess.'); end
cleanup = onCleanup(@() fclose(fid));
names = {'x','y','theta','v','a','phy','w','AX','AY','BX','BY','CX','CY','DX','DY'};
front = params_.vehicle.lw+params_.vehicle.lf;
rear = params_.vehicle.lr; half_width = params_.vehicle.lb/2;
c = cos(theta); s = sin(theta);
values = [x;y;theta;v;a;phy;w; ...
    x+front*c-half_width*s; y+front*s+half_width*c; ...
    x+front*c+half_width*s; y+front*s-half_width*c; ...
    x-rear*c+half_width*s; y-rear*s-half_width*c; ...
    x-rear*c-half_width*s; y-rear*s+half_width*c];
if any(~isfinite(values(:))) || ~isfinite(terminal_time)
    error('KBS2015:InitialGuess','The initial guess contains nonfinite values.');
end
for ii = 1 : params_.opti.nfe
    for jj = 1 : numel(names)
        fprintf(fid,'let %s[%d] := %.17g;\n',names{jj},ii,values(jj,ii));
    end
end
fprintf(fid,'let tf := %.17g;\n',terminal_time);
end
