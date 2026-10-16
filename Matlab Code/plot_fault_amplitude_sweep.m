function sweepTable = plot_fault_amplitude_sweep(amplitudes,scenario)
%PLOT_FAULT_AMPLITUDE_SWEEP Compare proposed and uncompensated control.
%
% Examples:
%   plot_fault_amplitude_sweep
%   plot_fault_amplitude_sweep([0 0.4 0.7 1.0 1.3],2)
%   T = plot_fault_amplitude_sweep(0:0.2:1.4,4)
%
% scenario=2 isolates the actuator fault and is recommended for the thesis.
% scenario=4 repeats the sweep with simultaneous actuator and sensor faults.

if nargin < 1 || isempty(amplitudes)
    amplitudes = [0 0.4 0.7 1.0 1.3];
end
if nargin < 2 || isempty(scenario)
    scenario = 2;
end
amplitudes = amplitudes(:);
if any(amplitudes < 0) || any(~isfinite(amplitudes))
    error('Fault amplitudes must be finite and nonnegative.');
end
if ~ismember(scenario,[2 4])
    error('Use scenario 2 (actuator fault) or scenario 4 (simultaneous faults).');
end

close all;
p = system_parameters();
check_assumptions(p);
obs = observer_lmi_design(p);
adp = adp_training(p);

% Preserve the shape of the fault while scaling its full magnitude.
% The original example used ripple/amplitude = 0.10/0.70.
rippleFraction = 0.10/0.70;

nA = numel(amplitudes);
ISE_Proposed = zeros(nA,1);
ISE_NoCompensation = zeros(nA,1);
JT_Proposed = zeros(nA,1);
JT_NoCompensation = zeros(nA,1);
InsideDomain_Proposed = false(nA,1);
InsideDomain_NoCompensation = false(nA,1);
Saturated_Proposed = false(nA,1);
Saturated_NoCompensation = false(nA,1);

for k = 1:nA
    pk = p;
    pk.fault.actuatorAmplitude = amplitudes(k);
    pk.fault.actuatorRipple = rippleFraction*amplitudes(k);

    proposed = run_simulation(pk,obs,adp,scenario,'proposed');
    noComp = run_simulation(pk,obs,adp,scenario,'no_compensation');

    ISE_Proposed(k) = trapz(proposed.t,sum(proposed.x.^2,2));
    ISE_NoCompensation(k) = trapz(noComp.t,sum(noComp.x.^2,2));
    JT_Proposed(k) = proposed.accumulatedCost(end);
    JT_NoCompensation(k) = noComp.accumulatedCost(end);
    InsideDomain_Proposed(k) = proposed.allInsideDomain;
    InsideDomain_NoCompensation(k) = noComp.allInsideDomain;
    Saturated_Proposed(k) = proposed.inputSaturated;
    Saturated_NoCompensation(k) = noComp.inputSaturated;

    fprintf(['Amplitude %.3f: ISE proposed=%.5f, no compensation=%.5f; ', ...
        'J_T proposed=%.5f, no compensation=%.5f\n'], ...
        amplitudes(k),ISE_Proposed(k),ISE_NoCompensation(k), ...
        JT_Proposed(k),JT_NoCompensation(k));
end

ISE_Reduction_Percent = 100*(ISE_NoCompensation-ISE_Proposed) ./ ...
    max(ISE_NoCompensation,eps);
JT_Reduction_Percent = 100*(JT_NoCompensation-JT_Proposed) ./ ...
    max(JT_NoCompensation,eps);

FaultAmplitude = amplitudes;
sweepTable = table(FaultAmplitude,ISE_Proposed,ISE_NoCompensation, ...
    JT_Proposed,JT_NoCompensation,ISE_Reduction_Percent, ...
    JT_Reduction_Percent,InsideDomain_Proposed, ...
    InsideDomain_NoCompensation,Saturated_Proposed, ...
    Saturated_NoCompensation);

disp('Fault-amplitude sweep results');
disp(sweepTable);

outputDir = fullfile(p.projectRoot,'results','amplitude_sweep');
if ~exist(outputDir,'dir')
    mkdir(outputDir);
end

f = figure('Visible','off','Color','w','Renderer','painters', ...
    'WindowStyle','normal','Units','pixels','Position',[100 100 1100 650]);
tl = tiledlayout(1,2,'TileSpacing','compact','Padding','compact');

nexttile;
plot(FaultAmplitude,ISE_Proposed,'o-','LineWidth',1.6,'MarkerSize',6); hold on;
plot(FaultAmplitude,ISE_NoCompensation,'s--','LineWidth',1.6,'MarkerSize',6);
grid on;
xlabel('Actuator fault amplitude');
ylabel('ISE_x');
legend('proposed','without compensation','Location','northwest', ...
    'Interpreter','none');
title('State regulation performance');

nexttile;
plot(FaultAmplitude,JT_Proposed,'o-','LineWidth',1.6,'MarkerSize',6); hold on;
plot(FaultAmplitude,JT_NoCompensation,'s--','LineWidth',1.6,'MarkerSize',6);
grid on;
xlabel('Actuator fault amplitude');
ylabel('J_T');
legend('proposed','without compensation','Location','northwest', ...
    'Interpreter','none');
title('Finite-horizon cost');

title(tl,sprintf('Fault-amplitude sensitivity, scenario %d',scenario));
format_figure(f);
pngFile = fullfile(outputDir,sprintf('fault_amplitude_sweep_scenario_%d.png',scenario));
print(f,pngFile,'-dpng','-r300');
close(f);

csvFile = fullfile(outputDir,sprintf('fault_amplitude_sweep_scenario_%d.csv',scenario));
matFile = fullfile(outputDir,sprintf('fault_amplitude_sweep_scenario_%d.mat',scenario));
writetable(sweepTable,csvFile);
save(matFile,'sweepTable','scenario','amplitudes');

if any(~InsideDomain_Proposed) || any(~InsideDomain_NoCompensation)
    warning('At least one run left the certified design domain.');
end
if any(Saturated_Proposed) || any(Saturated_NoCompensation)
    warning('At least one run encountered input saturation.');
end
fprintf('Figure saved to:\n%s\n',pngFile);
fprintf('Numerical table saved to:\n%s\n',csvFile);
end

function format_figure(f)
set(findall(f,'Type','axes'),'FontName','Times New Roman', ...
    'FontSize',11,'LineWidth',0.8);
set(findall(f,'Type','legend'),'FontName','Times New Roman', ...
    'FontSize',10,'Interpreter','none');
set(findall(f,'Type','text'),'FontName','Times New Roman', ...
    'FontSize',12,'Interpreter','none');
end
