function report = check_assumptions(p)
%CHECK_ASSUMPTIONS Numerically checks the structural assumptions in Chapter 3.

tol = 1e-9;
report.rankDs = rank(p.Ds,tol);
report.sensorFullColumnRank = report.rankDs == p.ns;
report.outputRedundancy = p.ny > p.ns;
report.leftNullResidual = norm(p.Ns*p.Ds,2);
report.leftNullCondition = report.leftNullResidual < 1e-10;

projectionResidual = p.Da-p.B*pinv(p.B)*p.Da;
report.matchingResidual = norm(projectionResidual,2);
report.matchingCondition = report.matchingResidual < 1e-10;

report.rankC0Da = rank(p.C0*p.Da,tol);
report.adaptiveGainNecessaryCondition = report.rankC0Da == p.na;

% If C0 has full column rank, the Rosenbrock matrix has full column rank
% for every complex s. Otherwise, the finite grid below is only diagnostic.
report.C0FullColumnRank = rank(p.C0,tol) == p.nx;
sigmas = linspace(-8,8,17);
omegas = linspace(-12,12,25);
minSigma = inf;
minRank = p.nx+p.na;
for a = sigmas
    for b = omegas
        s = a+1i*b;
        Rz = [s*eye(p.nx)-p.A, -p.Da; p.C0, zeros(size(p.C0,1),p.na)];
        sv = svd(Rz);
        minSigma = min(minSigma,min(sv));
        minRank = min(minRank,rank(Rz,tol));
    end
end
report.sampledRosenbrockMinSigma = minSigma;
report.sampledRosenbrockMinRank = minRank;
report.sampledNoInvariantZero = minRank == p.nx+p.na;

names = {'rank(Ds)=ns';'ny>ns';'Ns*Ds=0';'im(Da) subset im(B)'; ...
    'rank(C0*Da)=na';'sampled Rosenbrock full rank'};
passed = [report.sensorFullColumnRank;report.outputRedundancy; ...
    report.leftNullCondition;report.matchingCondition; ...
    report.adaptiveGainNecessaryCondition;report.sampledNoInvariantZero];
values = [report.rankDs;p.ny;report.leftNullResidual; ...
    report.matchingResidual;report.rankC0Da;report.sampledRosenbrockMinSigma];
report.table = table(names,passed,values, ...
    'VariableNames',{'Condition','Passed','DiagnosticValue'});

disp('Structural checks');
disp(report.table);
if ~all(passed)
    warning('At least one structural condition failed. Do not continue blindly.');
end
end
