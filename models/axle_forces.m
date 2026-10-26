function [Fyf, Fyr, af, ar] = axle_forces(vx, vy, r, delta, p)
%AXLE_FORCES Lateral axle forces of the single-track model (Fiala tyres).
vxs = max(vx, 1.0);
af = delta - atan2(vy + p.a*r, vxs);
ar = -atan2(vy - p.b*r, vxs);
Fzf = p.m*p.g*p.b/p.L;
Fzr = p.m*p.g*p.a/p.L;
Fyf = tire_fiala(af, p.Cf, Fzf, p.mu);
Fyr = tire_fiala(ar, p.Cr, Fzr, p.mu);
end
