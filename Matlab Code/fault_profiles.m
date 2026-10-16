function [fa,fs] = fault_profiles(t,scenario,p)
%FAULT_PROFILES Actuator and sensor faults for the four Chapter 4 scenarios.
% scenario=1: fault free
% scenario=2: actuator fault only
% scenario=3: sensor fault only
% scenario=4: simultaneous actuator and sensor faults

fa = zeros(p.na,1);
fs = zeros(p.ns,1);

% Smooth actuator fault: differentiable with a bounded rate of change.
k = p.fault.smoothness;
s = @(tau) 0.5*(1+tanh(k*tau));
window = s(t-p.fault.actuatorStart)-s(t-p.fault.actuatorEnd);
faProfile = p.fault.actuatorAmplitude*window + ...
    p.fault.actuatorRipple*sin(0.8*(t-p.fault.actuatorStart))*window;

% Piecewise-continuous sensor bias; jumps are permitted by the sensor model.
if t < p.fault.sensorStart
    fsProfile = 0;
elseif t < p.fault.sensorChange
    fsProfile = p.fault.sensorBias1;
else
    fsProfile = p.fault.sensorBias2;
end

switch scenario
    case 1
        % fault-free case
    case 2
        fa(1) = faProfile;
    case 3
        fs(1) = fsProfile;
    case 4
        fa(1) = faProfile;
        fs(1) = fsProfile;
    otherwise
        error('fault_profiles:UnknownScenario','Scenario must be 1, 2, 3 or 4.');
end
end
