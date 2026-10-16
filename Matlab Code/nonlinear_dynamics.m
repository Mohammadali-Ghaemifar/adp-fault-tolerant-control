function gx = nonlinear_dynamics(x,p)
%NONLINEAR_DYNAMICS Known nonlinear term g(x).
% For the demonstration system, |sin(a)-sin(b)| <= |a-b|, so gamma_g=0.25.

x = x(:);
gx = [0; 0.25*sin(x(1))];

if numel(gx) ~= p.nx
    error('nonlinear_dynamics:DimensionMismatch', ...
        'g(x) must have %d elements.',p.nx);
end
end
