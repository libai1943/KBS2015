function DrawResults()
global params_
r = params_.kbs; initial = params_.ha_result;
delete(findall(0,'Type','figure','Tag','KBS2015Trajectory'));
delete(findall(0,'Type','figure','Tag','KBS2015Profiles'));
figure('Color','w','Name',sprintf('Case %d - trajectory and footprints',params_.user.case_id), ...
    'NumberTitle','off','Tag','KBS2015Trajectory','Position',[80,140,1000,620]);
DrawParkingScenario();
% Footprints first. Both trajectories stay visible above all footprint lines.
h_footprints = DrawTrajFootprints(r.x,r.y,r.theta);
h_initial = plot(initial.x,initial.y,'--','Color',[0.12,0.12,0.12],'LineWidth',1.7,'DisplayName','Hybrid A* initial trajectory');
h_optimal = plot(r.x,r.y,'Color',[0.85,0.12,0.12],'LineWidth',2.2,'DisplayName','Optimized trajectory');
plot(r.x(1),r.y(1),'o','Color',[0.1,0.1,0.1],'MarkerFaceColor','w','HandleVisibility','off');
plot(r.x(end),r.y(end),'s','Color',[0.85,0.12,0.12],'MarkerFaceColor','w','HandleVisibility','off');
quiver(r.x([1,end]),r.y([1,end]),cos(r.theta([1,end])),sin(r.theta([1,end])),0, ...
    'Color',[0.1,0.1,0.1],'LineWidth',1.3,'MaxHeadSize',0.7,'HandleVisibility','off');
axis tight; limits = axis; axis(limits+[-1.2,1.2,-1.2,1.2]);
set(gca,'FontSize',12,'Layer','top');
title(sprintf('Case %d | T = %.4f s',params_.user.case_id,r.terminal_time));
legend([h_optimal,h_initial,h_footprints],'Location','northoutside','Orientation','horizontal');

figure('Color','w','Name',sprintf('Case %d - optimal states and controls',params_.user.case_id), ...
    'NumberTitle','off','Tag','KBS2015Profiles','Position',[130,90,1100,700]);
tiledlayout(2,3,'TileSpacing','compact','Padding','compact');
t = linspace(0,r.terminal_time,numel(r.x));
names = {'v','phy','theta','a','w'};
labels = {'v / (m s^{-1})','\phi / rad','\theta / rad','a / (m s^{-2})','\omega / (rad s^{-1})'};
titles = {'Speed','Steering angle','Heading','Acceleration','Steering rate'};
for ii = 1 : 5
    nexttile(ii);
    if ii <= 3
        plot(t,r.(names{ii}),'k','LineWidth',1.7);
    else
        stairs(t,r.(names{ii}),'Color',[0.85,0.12,0.12],'LineWidth',1.5);
    end
    grid on; box on; xlim([0,r.terminal_time]);
    xlabel('t / s'); ylabel(labels{ii}); title(titles{ii}); set(gca,'FontSize',11);
end
nexttile(6); plot(t,r.x,'LineWidth',1.7); hold on; plot(t,r.y,'LineWidth',1.7);
grid on; box on; xlim([0,r.terminal_time]); xlabel('t / s'); ylabel('Position / m');
title('Rear-axle centre'); legend('x','y','Location','best'); set(gca,'FontSize',11);
sgtitle(sprintf('Case %d | Optimal states and controls | T = %.4f s',params_.user.case_id,r.terminal_time));
drawnow;
end
