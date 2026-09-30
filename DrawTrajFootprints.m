function handle = DrawTrajFootprints(x,y,theta)
global params_
% Evenly spaced in travelled distance, including the initial and final pose.
s = [0,cumsum(hypot(diff(x),diff(y)))];
[distance,index] = unique(s,'stable');
samples = unique(round(interp1(distance,index,linspace(0,s(end),45))));
for ii = samples
    V = CreateVehiclePolygon(x(ii),y(ii),theta(ii),2);
    handle = plot(V.x,V.y,'Color',params_.utility.ego_vehicle_rgb,'LineWidth',0.65,'HandleVisibility','off');
end
set(handle,'HandleVisibility','on','DisplayName','Optimal footprints');
end
