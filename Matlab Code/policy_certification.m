function [acceptedPolicy,beta,stats] = policy_certification(oldPolicy,candidatePolicy,p)
%POLICY_CERTIFICATION Numerical admissibility screening on the design grid.
% This finite-grid test is a simulation diagnostic, not a uniform proof.

axesData = cell(p.nx,1);
for j = 1:p.nx
    axesData{j} = linspace(p.domain.lower(j),p.domain.upper(j), ...
        p.adp.gridPointsPerState);
end
if p.nx ~= 2
    error('The demonstration grid generator currently supports nx=2.');
end
[X1,X2] = ndgrid(axesData{1},axesData{2});
X = [X1(:)';X2(:)'];
X(:,vecnorm(X,2,1)<1e-9) = [];

betas = 1;
while betas(end) > p.adp.minimumBeta
    betas(end+1) = betas(end)/2; %#ok<AGROW>
end

acceptedPolicy = oldPolicy;
beta = 0;
stats.minimumDecayMargin = -inf;
stats.maximumGainRatio = inf;
stats.passed = false;

for b = betas
    trial = @(x) (1-b)*oldPolicy(x)+b*candidatePolicy(x);
    margin = inf;
    gainRatio = 0;
    for k = 1:size(X,2)
        x = X(:,k);
        u = trial(x);
        f = p.A*x+nonlinear_dynamics(x,p)+p.B*u;
        dV = 2*x'*p.adp.Pcert*f;
        margin = min(margin,-dV/(x'*x));
        gainRatio = max(gainRatio,norm(u)/norm(x));
    end
    if margin >= p.adp.acceptanceMargin && isfinite(gainRatio)
        acceptedPolicy = trial;
        beta = b;
        stats.minimumDecayMargin = margin;
        stats.maximumGainRatio = gainRatio;
        stats.passed = true;
        return;
    end
end
end
