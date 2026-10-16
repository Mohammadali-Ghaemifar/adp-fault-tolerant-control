function main_chapter4()
%MAIN_CHAPTER4 Complete numerical workflow for the proposed Chapter 4.

close all; clc;
rng(11,'twister');
p = system_parameters();

checks = check_assumptions(p);
obs = observer_lmi_design(p);
adp = adp_training(p);

% ADP training diagnostics
plot_adp_history(adp,p);

results = cell(4,1);
metrics = cell(4,1);
for scenario = 1:4
    results{scenario} = run_simulation(p,obs,adp,scenario,'proposed');
    metrics{scenario} = compute_metrics(results{scenario},p);
    plot_chapter4_results(results{scenario},p);
end

% Baseline comparison for the two most important faulty cases.
comparisonResults = struct();
for scenario = [2 4]
    field = sprintf('scenario%d',scenario);
    comparisonResults.(field).withoutCompensation = ...
        run_simulation(p,obs,adp,scenario,'no_compensation');
    comparisonResults.(field).ideal = ...
        run_simulation(p,obs,adp,scenario,'ideal');
    plot_comparison(results{scenario}, ...
        comparisonResults.(field).withoutCompensation, ...
        comparisonResults.(field).ideal,p);
end

summary = metrics_table(metrics);
disp('Chapter 4 quantitative summary');
disp(summary);
writetable(summary,fullfile(p.outputDir,'chapter4_metrics.csv'));
save(fullfile(p.outputDir,'chapter4_results.mat'), ...
    'p','checks','obs','adp','results','comparisonResults','summary','-v7.3');
fprintf('All outputs were saved in:\n%s\n',p.outputDir);
end

function T = metrics_table(metrics)
n = numel(metrics);
Scenario = zeros(n,1);
Mode = strings(n,1);
RMSE_x = zeros(n,1);
RMSE_fa = zeros(n,1);
RMSE_fs = zeros(n,1);
State_ISE = zeros(n,1);
Control_Energy = zeros(n,1);
J_T = zeros(n,1);
Peak_State = zeros(n,1);
Peak_Control = zeros(n,1);
Settling_Time = zeros(n,1);
Inside_Domain = false(n,1);
for k = 1:n
    m = metrics{k};
    Scenario(k) = m.scenario;
    Mode(k) = m.mode;
    RMSE_x(k) = m.rmseStateEstimation;
    RMSE_fa(k) = m.rmseActuatorFault;
    RMSE_fs(k) = m.rmseSensorFault;
    State_ISE(k) = m.stateISE;
    Control_Energy(k) = m.controlEnergy;
    J_T(k) = m.finiteHorizonCost;
    Peak_State(k) = m.peakStateNorm;
    Peak_Control(k) = m.peakControlNorm;
    Settling_Time(k) = m.settlingTime;
    Inside_Domain(k) = m.allInsideDomain;
end
T = table(Scenario,Mode,RMSE_x,RMSE_fa,RMSE_fs,State_ISE, ...
    Control_Energy,J_T,Peak_State,Peak_Control,Settling_Time,Inside_Domain);
end

function plot_adp_history(adp,p)
iterations = 1:adp.iterations;
f = figure('Visible','off','Color','w','Renderer','painters','WindowStyle','normal');
tl = tiledlayout(2,2,'TileSpacing','compact','Padding','compact');
nexttile; plot(iterations,adp.history.weights','LineWidth',1.1); grid on;
xlabel('Iteration'); ylabel('Critic weights');
nexttile; semilogy(iterations,adp.history.rmseBellman,'o-','LineWidth',1.2); grid on;
xlabel('Iteration'); ylabel('Bellman RMSE');
nexttile; semilogy(iterations,adp.history.sigmaMin,'o-','LineWidth',1.2); grid on;
xlabel('Iteration'); ylabel('Minimum singular value');
nexttile; stairs(iterations,adp.history.beta,'o-','LineWidth',1.2); grid on;
xlabel('Iteration'); ylabel('Accepted beta'); ylim([0 1.05]);
title(tl,'Offline ADP training diagnostics');
save_figure(f,fullfile(p.outputDir,'adp_training_diagnostics.png'));
close(f);
end

function plot_comparison(proposed,noComp,ideal,p)
t = proposed.t;
f = figure('Visible','off','Color','w','Renderer','painters','WindowStyle','normal');
tl = tiledlayout(2,1,'TileSpacing','compact','Padding','compact');
nexttile;
plot(t,vecnorm(proposed.x,2,2),'LineWidth',1.4); hold on;
plot(t,vecnorm(noComp.x,2,2),'--','LineWidth',1.3);
plot(t,vecnorm(ideal.x,2,2),':','LineWidth',1.5);
grid on; ylabel('||x||');
legend('proposed','without compensation','ideal information','Location','best');
nexttile;
plot(t,vecnorm(proposed.u,2,2),'LineWidth',1.4); hold on;
plot(t,vecnorm(noComp.u,2,2),'--','LineWidth',1.3);
plot(t,vecnorm(ideal.u,2,2),':','LineWidth',1.5);
grid on; ylabel('||u||'); xlabel('Time (s)');
title(tl,sprintf('Scenario %d comparison',proposed.scenario));
name = sprintf('comparison_scenario_%d.png',proposed.scenario);
save_figure(f,fullfile(p.outputDir,name));
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
