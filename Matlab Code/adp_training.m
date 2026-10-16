function adp = adp_training(p)
%ADP_TRAINING Offline model-based policy iteration using collocation points.

if p.nx ~= 2 || p.nu ~= 1
    error(['The supplied critic basis is written for nx=2 and nu=1. ', ...
        'Replace critic_features and critic_jacobian for another dimension.']);
end

Acl0 = p.A-p.B*p.adp.initialGain;
p.adp.Pcert = local_lyapunov_solution(Acl0,eye(p.nx));

q1 = linspace(p.domain.lower(1),p.domain.upper(1),p.adp.gridPointsPerState);
q2 = linspace(p.domain.lower(2),p.domain.upper(2),p.adp.gridPointsPerState);
[X1,X2] = ndgrid(q1,q2);
X = [X1(:)';X2(:)'];
X(:,vecnorm(X,2,1)<1e-9) = [];

policy = @(x) -p.adp.initialGain*x(:);
Wprevious = [];
history.weights = [];
history.rmseBellman = [];
history.rank = [];
history.sigmaMin = [];
history.conditionNumber = [];
history.beta = [];
history.decayMargin = [];
history.accepted = [];

for iteration = 1:p.adp.maxIterations
    sampleFeature = critic_features(X(:,1),p);
    Nc = numel(sampleFeature);
    Omega = zeros(size(X,2),Nc);
    b = zeros(size(X,2),1);

    for k = 1:size(X,2)
        x = X(:,k);
        u = policy(x);
        Jsigma = critic_jacobian(x,p);
        fPolicy = p.A*x+nonlinear_dynamics(x,p)+p.B*u;
        Omega(k,:) = (Jsigma*fPolicy)';
        b(k) = x'*p.Qc*x+u'*p.R*u;
    end

    singularValues = svd(Omega,'econ');
    numericalRank = sum(singularValues > ...
        p.adp.rankTolerance*max(singularValues));
    sigmaMin = min(singularValues);
    conditionNumber = max(singularValues)/max(sigmaMin,eps);
    if numericalRank < Nc
        error('ADP regression is rank deficient: rank=%d, Nc=%d.',numericalRank,Nc);
    end

    W = -Omega\b;
    bellmanResidual = Omega*W+b;
    candidate = @(x) critic_policy(x,W,p);
    [newPolicy,beta,cert] = policy_certification(policy,candidate,p);

    history.weights(:,iteration) = W; %#ok<AGROW>
    history.rmseBellman(iteration,1) = sqrt(mean(bellmanResidual.^2));
    history.rank(iteration,1) = numericalRank;
    history.sigmaMin(iteration,1) = sigmaMin;
    history.conditionNumber(iteration,1) = conditionNumber;
    history.beta(iteration,1) = beta;
    history.decayMargin(iteration,1) = cert.minimumDecayMargin;
    history.accepted(iteration,1) = cert.passed;

    fprintf(['ADP iteration %2d: Bellman RMSE=%.3e, rank=%d, ', ...
        'sigmaMin=%.3e, beta=%.4f\n'],iteration, ...
        history.rmseBellman(iteration),numericalRank,sigmaMin,beta);

    policy = newPolicy;
    if ~isempty(Wprevious) && iteration >= p.adp.minIterations
        if norm(W-Wprevious) <= p.adp.weightTolerance*(1+norm(Wprevious))
            break;
        end
    end
    Wprevious = W;
end

adp.policy = policy;
adp.lastWeights = W;
adp.history = history;
adp.iterations = size(history.weights,2);
adp.collocationPoints = X;
adp.Pcert = p.adp.Pcert;
adp.basisCount = size(history.weights,1);
adp.finalPolicyIsBlended = history.beta(end) < 1;
end

function sigma = critic_features(x,p)
x = x(:);
s = max(abs([p.domain.lower,p.domain.upper]),[],2);
z1 = x(1)/s(1);
z2 = x(2)/s(2);
sigma = [z1^2;z1*z2;z2^2;z1^4;z1^3*z2; ...
    z1^2*z2^2;z1*z2^3;z2^4];
end

function J = critic_jacobian(x,p)
x = x(:);
s = max(abs([p.domain.lower,p.domain.upper]),[],2);
z1 = x(1)/s(1);
z2 = x(2)/s(2);
d1 = 1/s(1);
d2 = 1/s(2);
J = [2*z1*d1,                         0; ...
     z2*d1,                       z1*d2; ...
     0,                         2*z2*d2; ...
     4*z1^3*d1,                     0; ...
     3*z1^2*z2*d1,             z1^3*d2; ...
     2*z1*z2^2*d1,       2*z1^2*z2*d2; ...
     z2^3*d1,               3*z1*z2^2*d2; ...
     0,                         4*z2^3*d2];
end

function u = critic_policy(x,W,p)
J = critic_jacobian(x,p);
u = -0.5*(p.R\(p.B'*J'*W));
end

function P = local_lyapunov_solution(A,Q)
n = size(A,1);
Kron = kron(eye(n),A')+kron(A',eye(n));
P = reshape(-Kron\Q(:),n,n);
P = (P+P')/2;
if min(eig(P)) <= 0
    error('Initial policy is not certified by a positive Lyapunov matrix.');
end
end
