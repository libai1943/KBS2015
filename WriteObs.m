function WriteObs()
global params_
fid = fopen(fullfile('AmplInputs','ObstacleVertices.txt'),'w');
if fid < 0, error('KBS2015:FileWrite','Cannot write obstacle vertices.'); end
cleanup = onCleanup(@() fclose(fid));
for ii = 1 : params_.obstacle.num_obs
    V = params_.obstacle.obs{ii};
    for jj = 1 : 4
        fprintf(fid,'%d %d 1 %.17g\n',ii,jj,V.x(jj));
        fprintf(fid,'%d %d 2 %.17g\n',ii,jj,V.y(jj));
    end
end
end
