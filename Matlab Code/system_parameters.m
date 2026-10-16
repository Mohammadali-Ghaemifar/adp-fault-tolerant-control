function p = system_parameters()
%SYSTEM_PARAMETERS Numerical data used by all Chapter 4 simulations.
% Replace only the plant-specific block below when the real benchmark is known.

%% Demonstration plant: nx=2, nu=1, ny=3, na=1, ns=1
p.A  = [0 1; -2 -0.8];
p.B  = [0; 1];
p.C  = [1 0; 0 1; 1 1];
p.Da = [0; 1];
p.Ds = [0; 0; 1];

% Left-null-space projection of Ds. Here the first two outputs are fault-free.
p.Ns = [1 0 0; 0 1 0];
p.C0 = p.Ns*p.C;

p.nx = size(p.A,1);
p.nu = size(p.B,2);
p.ny = size(p.C,1);
p.na = size(p.Da,2);
p.ns = size(p.Ds,2);

%% Nonlinearity and local design domain
p.gamma_g = 0.25;              % Lipschitz constant for g(x) used here
p.domain.lower = [-2; -2];
p.domain.upper = [ 2;  2];

%% Observer settings
p.observer.Gamma = 3*eye(p.na);
p.observer.delta = 0.25;
p.observer.epsilon = 1e-7;
p.observer.fallbackAlpha = 2;  % used only when YALMIP is unavailable

%% ADP settings
p.Qc = diag([4 1]);
p.R  = 0.5*eye(p.nu);
p.adp.initialGain = [1.6 1.2]; % mu_0(x)=-K0*x, stabilizing for this example
p.adp.gridPointsPerState = 17;
p.adp.maxIterations = 20;
p.adp.minIterations = 3;
p.adp.weightTolerance = 1e-5;
p.adp.acceptanceMargin = 1e-3;
p.adp.minimumBeta = 1/64;
p.adp.rankTolerance = 1e-9;

%% Time-domain simulation settings
p.sim.t0 = 0;
p.sim.tf = 25;
p.sim.outputStep = 0.01;
p.sim.x0 = [1.2; -0.7];
p.sim.xhat0 = [0; 0];
p.sim.fahat0 = zeros(p.na,1);
p.sim.relativeTolerance = 1e-7;
p.sim.absoluteTolerance = 1e-9;
p.sim.maxStep = 0.02;
p.sim.uMax = inf;              % theorem assumes no saturation
p.sim.noiseStd = zeros(p.ny,1);% keep zero for the theorem-consistent runs
p.sim.settlingTolerance = 0.05;

%% Fault profiles
p.fault.actuatorStart = 5;
p.fault.actuatorEnd = 17;
p.fault.actuatorAmplitude = 1;
p.fault.actuatorRipple = 0.10;
p.fault.smoothness = 3.0;
p.fault.sensorStart = 8;
p.fault.sensorChange = 16;
p.fault.sensorBias1 = 0.45;
p.fault.sensorBias2 = 0.20;

%% Optional theoretical bound, if computed from Chapter 3 constants
p.theory.rc = NaN;

p.projectRoot = fileparts(mfilename('fullpath'));
p.outputDir = fullfile(p.projectRoot,'results');
if ~exist(p.outputDir,'dir')
    mkdir(p.outputDir);
end
end
