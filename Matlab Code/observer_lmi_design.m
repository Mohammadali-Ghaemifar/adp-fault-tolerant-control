function obs = observer_lmi_design(p)
%OBSERVER_LMI_DESIGN Solves the observer LMI and adaptive-gain equality.
% YALMIP is used when available. A verified fallback is supplied only for
% the demonstration plant in system_parameters.m.

nx = p.nx;
p0 = size(p.C0,1);
eps0 = p.observer.epsilon;
usedFallback = false;

if exist('sdpvar','file') == 2
    try
        P = sdpvar(nx,nx,'symmetric');
        Y = sdpvar(nx,p0,'full');
        K = sdpvar(p.na,p0,'full');
        rho = sdpvar(1,1);

        Pi = p.A'*P+P*p.A-p.C0'*Y'-Y*p.C0 + ...
            rho*p.gamma_g^2*eye(nx);
        M = [Pi, P; P, -rho*eye(nx)];
        constraints = [P >= eps0*eye(nx), rho >= eps0, ...
            M <= -eps0*eye(2*nx), K*p.C0 == p.Da'*P];
        if exist('sqlp','file') == 2
            options = sdpsettings('solver','sdpt3','verbose',0);
        else
            options = sdpsettings('verbose',0);
        end
        diagnostics = optimize(constraints,[],options);
        if diagnostics.problem ~= 0
            error('YALMIP failed: %s',diagnostics.info);
        end
        obs.P = value(P);
        obs.Y = value(Y);
        obs.K = value(K);
        obs.rho = value(rho);
        obs.solverInfo = diagnostics.info;
    catch ME
        warning('observer_lmi_design:YALMIPFailure', ...
            'LMI solution failed (%s). Using the demonstration fallback.',ME.message);
        usedFallback = true;
    end
else
    usedFallback = true;
end

if usedFallback
    if rank(p.C0) < nx
        error(['The fallback requires C0 to have full column rank. ', ...
            'Install YALMIP and an SDP solver for the real plant.']);
    end
    obs.P = eye(nx);
    obs.rho = 1;
    obs.L = p.A+p.observer.fallbackAlpha*eye(nx);
    obs.Y = obs.P*obs.L;
    obs.K = (p.Da'*obs.P)*pinv(p.C0);
    obs.solverInfo = 'Analytical fallback for the supplied demonstration plant';
else
    obs.L = obs.P\obs.Y;
end

obs.Gamma = p.observer.Gamma;
obs.delta = p.observer.delta;
obs.usedFallback = usedFallback;

PiNum = p.A'*obs.P+obs.P*p.A-p.C0'*obs.Y'-obs.Y*p.C0 + ...
    obs.rho*p.gamma_g^2*eye(nx);
obs.lmiMatrix = [PiNum, obs.P; obs.P, -obs.rho*eye(nx)];
obs.maxLmiEigenvalue = max(real(eig((obs.lmiMatrix+obs.lmiMatrix')/2)));
obs.minPEigenvalue = min(real(eig((obs.P+obs.P')/2)));
obs.gainEqualityResidual = norm(obs.K*p.C0-p.Da'*obs.P,'fro');
obs.feasible = obs.maxLmiEigenvalue < -1e-8 && ...
    obs.minPEigenvalue > 0 && obs.gainEqualityResidual < 1e-7;

fprintf('Observer LMI: max eigenvalue = %.3e\n',obs.maxLmiEigenvalue);
fprintf('Adaptive equality residual = %.3e\n',obs.gainEqualityResidual);
if ~obs.feasible
    error('Observer design did not pass the numerical feasibility checks.');
end
end
