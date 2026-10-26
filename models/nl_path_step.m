function xi = nl_path_step(xi, u, vx, kappa, p, Ts, substeps)
%NL_PATH_STEP Nonlinear prediction model of the NMPC, in path coordinates.
%   xi = [e_y; e_psi; vy; r; delta], u = steering command, vx = speed,
%   kappa = path curvature. Integrated with RK4 over Ts.
h = Ts/substeps;
for i = 1:substeps
    k1 = deriv(xi,          u, vx, kappa, p);
    k2 = deriv(xi + h/2*k1, u, vx, kappa, p);
    k3 = deriv(xi + h/2*k2, u, vx, kappa, p);
    k4 = deriv(xi + h*k3,   u, vx, kappa, p);
    xi = xi + h/6*(k1 + 2*k2 + 2*k3 + k4);
end
end

function d = deriv(xi, u, vx, kappa, p)
ey = xi(1); epsi = xi(2); vy = xi(3); r = xi(4); delta = xi(5);
[Fyf, Fyr] = axle_forces(vx, vy, r, delta, p);
sdot = (vx*cos(epsi) - vy*sin(epsi)) / (1 - kappa*ey);
d = [vx*sin(epsi) + vy*cos(epsi);
     r - kappa*sdot;
     (Fyf*cos(delta) + Fyr)/p.m - vx*r;
     (p.a*Fyf*cos(delta) - p.b*Fyr)/p.Iz;
     (u - delta)/p.tau_steer];
end
