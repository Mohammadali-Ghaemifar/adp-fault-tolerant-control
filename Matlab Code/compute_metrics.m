function metrics = compute_metrics(result,p)
%COMPUTE_METRICS Quantitative indices used in Chapter 4 tables.

t = result.t;
metrics.scenario = result.scenario;
metrics.mode = string(result.mode);
metrics.rmseStateEstimation = sqrt(mean(sum(result.ex.^2,2)));
metrics.rmseActuatorFault = active_rmse(result.efa,result.fa);
metrics.rmseSensorFault = active_rmse(result.efs,result.fs);
metrics.stateISE = trapz(t,sum(result.x.^2,2));
metrics.controlEnergy = trapz(t,sum(result.u.^2,2));
metrics.finiteHorizonCost = result.accumulatedCost(end);
metrics.peakStateNorm = max(vecnorm(result.x,2,2));
metrics.peakControlNorm = max(vecnorm(result.u,2,2));
metrics.finalStateNorm = norm(result.x(end,:));
metrics.allInsideDomain = result.allInsideDomain;
metrics.inputSaturated = result.inputSaturated;

stateNorm = vecnorm(result.x,2,2);
switch result.scenario
    case 1
        lastEvent = p.sim.t0;
    case 2
        lastEvent = p.fault.actuatorEnd;
    case 3
        lastEvent = p.fault.sensorChange;
    otherwise
        lastEvent = max([p.fault.actuatorEnd,p.fault.sensorChange]);
end
eligible = find(t >= lastEvent);
outside = eligible(stateNorm(eligible) > p.sim.settlingTolerance);
if isempty(outside)
    metrics.settlingTime = lastEvent;
elseif outside(end) < numel(t)
    metrics.settlingTime = t(outside(end)+1);
else
    metrics.settlingTime = NaN;
end

tail = max(1,round(0.8*numel(t))):numel(t);
metrics.observedUltimateW = max(result.wNorm(tail));
metrics.theoreticalRc = p.theory.rc;
end

function value = active_rmse(errorSignal,trueSignal)
active = vecnorm(trueSignal,2,2) > 1e-5;
if any(active)
    value = sqrt(mean(sum(errorSignal(active,:).^2,2)));
else
    value = sqrt(mean(sum(errorSignal.^2,2)));
end
end
