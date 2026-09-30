function DrawParkingScenario()
% Draw into the current axes; figure creation belongs to DrawResults.
global params_
hold on; box on; grid on; axis equal;
for ii = 1 : params_.obstacle.num_obs
    V = params_.obstacle.obs{ii};
    fill(V.x,V.y,[0.48,0.49,0.51],'EdgeColor',[0.35,0.36,0.38],'HandleVisibility','off');
end
xlabel('x / m'); ylabel('y / m');
end
