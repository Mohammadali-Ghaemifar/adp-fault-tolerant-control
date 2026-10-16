function plot_chapter4_results(result,p)
%PLOT_CHAPTER4_RESULTS Creates publication-ready figures for one scenario.

scenarioNames = {'fault_free','actuator_fault','sensor_fault','simultaneous_faults'};
tag = scenarioNames{result.scenario} + "_" + string(result.mode);
t = result.t;

%% Actual and estimated states
f = figure('Visible','off','Color','w','Renderer','painters','WindowStyle','normal');
tl = tiledlayout(p.nx,1,'TileSpacing','compact','Padding','compact');
for j = 1:p.nx
    nexttile;
    plot(t,result.x(:,j),'LineWidth',1.4); hold on;
    plot(t,result.xhat(:,j),'--','LineWidth',1.3);
    grid on; ylabel(sprintf('x_%d',j),'Interpreter','none');
    legend('actual','estimated','Location','best','Interpreter','none');
end
xlabel(tl,'Time (s)');
title(tl,'Actual and estimated states');
save_figure(f,fullfile(p.outputDir,"states_"+tag+".png"));
close(f);

%% Actual and reconstructed faults
f = figure('Visible','off','Color','w','Renderer','painters','WindowStyle','normal');
tl = tiledlayout(2,1,'TileSpacing','compact','Padding','compact');
nexttile;
plot(t,result.fa,'LineWidth',1.4); hold on;
plot(t,result.fahat,'--','LineWidth',1.3);
grid on; ylabel('f_a','Interpreter','none');
legend('actual','estimated','Location','best','Interpreter','none');
nexttile;
plot(t,result.fs,'LineWidth',1.4); hold on;
plot(t,result.fshat,'--','LineWidth',1.3);
grid on; ylabel('f_s','Interpreter','none'); xlabel('Time (s)');
legend('actual','reconstructed','Location','best','Interpreter','none');
title(tl,'Fault estimation and reconstruction');
save_figure(f,fullfile(p.outputDir,"faults_"+tag+".png"));
close(f);

%% Error norms and UUB diagnostic
f = figure('Visible','off','Color','w','Renderer','painters','WindowStyle','normal');
tl = tiledlayout(2,1,'TileSpacing','compact','Padding','compact');
nexttile;
semilogy(t,max(vecnorm(result.ex,2,2),1e-12),'LineWidth',1.4); hold on;
semilogy(t,max(vecnorm(result.efa,2,2),1e-12),'LineWidth',1.3);
semilogy(t,max(vecnorm(result.efs,2,2),1e-12),'LineWidth',1.3);
grid on; ylabel('Error norm');
legend('||e_x||','||e_fa||','||e_fs||','Location','best','Interpreter','none');
nexttile;
plot(t,result.wNorm,'LineWidth',1.4); hold on;
if isfinite(p.theory.rc)
    yline(p.theory.rc,'--r','r_c','LineWidth',1.2);
end
grid on; ylabel('||w||'); xlabel('Time (s)');
title(tl,'Estimation errors and ultimate-boundedness diagnostic');
save_figure(f,fullfile(p.outputDir,"errors_"+tag+".png"));
close(f);

%% Control components and measured/virtual outputs
f = figure('Visible','off','Color','w','Renderer','painters','WindowStyle','normal');
tl = tiledlayout(2,1,'TileSpacing','compact','Padding','compact');
nexttile;
plot(t,result.uNominal,'LineWidth',1.3); hold on;
plot(t,result.uCompensation,'--','LineWidth',1.3);
plot(t,result.u,'k','LineWidth',1.4);
grid on; ylabel('Control');
legend('ADP policy','fault compensation','total input','Location','best', ...
    'Interpreter','none');
nexttile;
plot(t,result.y,'LineWidth',1.0); hold on;
plot(t,result.y0,'--','LineWidth',1.3);
grid on; ylabel('Outputs'); xlabel('Time (s)');
title(tl,'Control signal and virtual output');
save_figure(f,fullfile(p.outputDir,"control_outputs_"+tag+".png"));
close(f);
end

function save_figure(f,fileName)
set(f,'Units','pixels','Position',[100 100 1000 700]);
set(findall(f,'Type','axes'),'FontName','Times New Roman','FontSize',10, ...
    'LineWidth',0.8);
set(findall(f,'Type','legend'),'FontName','Times New Roman','FontSize',9, ...
    'Interpreter','none');
set(findall(f,'Type','text'),'FontName','Times New Roman','FontSize',11, ...
    'Interpreter','none');
print(f,fileName,'-dpng','-r300');
end
