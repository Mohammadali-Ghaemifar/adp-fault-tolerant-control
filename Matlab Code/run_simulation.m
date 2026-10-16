function result = run_simulation(p,obs,adp,scenario,mode)
%RUN_SIMULATION Runs one scenario with a selected compensation mode.

tRequested = (p.sim.t0:p.sim.outputStep:p.sim.tf)';
if tRequested(end) < p.sim.tf
    tRequested(end+1,1) = p.sim.tf;
end
z0 = [p.sim.x0;p.sim.xhat0;p.sim.fahat0;0];
options = odeset('RelTol',p.sim.relativeTolerance, ...
    'AbsTol',p.sim.absoluteTolerance,'MaxStep',p.sim.maxStep);
rhs = @(t,z) closed_loop_dynamics(t,z,p,obs,adp,scenario,mode);
[t,z] = ode45(rhs,tRequested,z0,options);

N = numel(t);
nx = p.nx;
na = p.na;
x = z(:,1:nx);
xhat = z(:,nx+1:2*nx);
fahat = z(:,2*nx+1:2*nx+na);
fa = zeros(N,p.na);
fs = zeros(N,p.ns);
fshat = zeros(N,p.ns);
y = zeros(N,p.ny);
y0 = zeros(N,size(p.C0,1));
uNominal = zeros(N,p.nu);
uCompensation = zeros(N,p.nu);
u = zeros(N,p.nu);
wNorm = zeros(N,1);
insideDomain = false(N,1);

for k = 1:N
    xk = x(k,:)';
    xhk = xhat(k,:)';
    fahk = fahat(k,:)';
    [fak,fsk] = fault_profiles(t(k),scenario,p);
    yk = p.C*xk+p.Ds*fsk;
    y0k = p.Ns*yk;
    fshk = pinv(p.Ds)*(yk-p.C*xhk);
    unom = adp.policy(xhk);

    switch lower(string(mode))
        case "proposed"
            ucomp = -pinv(p.B)*p.Da*fahk;
        case "no_compensation"
            ucomp = zeros(p.nu,1);
        case "ideal"
            unom = adp.policy(xk);
            ucomp = -pinv(p.B)*p.Da*fak;
        otherwise
            error('run_simulation:UnknownMode','Unknown simulation mode.');
    end
    uk = unom+ucomp;
    if isfinite(p.sim.uMax)
        uk = min(max(uk,-p.sim.uMax),p.sim.uMax);
    end

    fa(k,:) = fak';
    fs(k,:) = fsk';
    fshat(k,:) = fshk';
    y(k,:) = yk';
    y0(k,:) = y0k';
    uNominal(k,:) = unom';
    uCompensation(k,:) = ucomp';
    u(k,:) = uk';
    ex = xhk-xk;
    efa = fahk-fak;
    wNorm(k) = norm([xk;ex;efa]);
    insideDomain(k) = all(xk >= p.domain.lower) && ...
        all(xk <= p.domain.upper) && all(xhk >= p.domain.lower) && ...
        all(xhk <= p.domain.upper);
end

result.scenario = scenario;
result.mode = char(mode);
result.t = t;
result.x = x;
result.xhat = xhat;
result.ex = xhat-x;
result.fa = fa;
result.fahat = fahat;
result.efa = fahat-fa;
result.fs = fs;
result.fshat = fshat;
result.efs = fshat-fs;
result.y = y;
result.y0 = y0;
result.uNominal = uNominal;
result.uCompensation = uCompensation;
result.u = u;
result.accumulatedCost = z(:,end);
result.wNorm = wNorm;
result.insideDomain = insideDomain;
result.allInsideDomain = all(insideDomain);
result.inputSaturated = isfinite(p.sim.uMax) && ...
    any(abs(u(:)) >= p.sim.uMax-1e-8);
end
