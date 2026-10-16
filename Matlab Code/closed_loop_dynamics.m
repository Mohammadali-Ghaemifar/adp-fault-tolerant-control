function dz = closed_loop_dynamics(t,z,p,obs,adp,scenario,mode)
%CLOSED_LOOP_DYNAMICS Plant, observer, adaptive law and accumulated cost.

nx = p.nx;
na = p.na;
x = z(1:nx);
xhat = z(nx+1:2*nx);
fahat = z(2*nx+1:2*nx+na);

[fa,fs] = fault_profiles(t,scenario,p);
y = p.C*x+p.Ds*fs;
y0 = p.Ns*y;
innovation = y0-p.C0*xhat;

uNominal = adp.policy(xhat);
switch lower(string(mode))
    case "proposed"
        uCompensation = -pinv(p.B)*p.Da*fahat;
    case "no_compensation"
        uCompensation = zeros(p.nu,1);
    case "ideal"
        uNominal = adp.policy(x);
        uCompensation = -pinv(p.B)*p.Da*fa;
    otherwise
        error('closed_loop_dynamics:UnknownMode','Unknown simulation mode.');
end
u = uNominal+uCompensation;
if isfinite(p.sim.uMax)
    u = min(max(u,-p.sim.uMax),p.sim.uMax);
end

xdot = p.A*x+p.B*u+nonlinear_dynamics(x,p)+p.Da*fa;
xhatdot = p.A*xhat+p.B*u+nonlinear_dynamics(xhat,p)+ ...
    p.Da*fahat+obs.L*innovation;
fahatdot = -obs.delta*obs.Gamma*fahat+obs.Gamma*obs.K*innovation;
costDot = x'*p.Qc*x+u'*p.R*u;

dz = [xdot;xhatdot;fahatdot;costDot];
end
