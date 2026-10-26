function dx = plant_deriv(x, delta_cmd, ax_cmd, p)
%PLANT_DERIV Nonlinear single-track vehicle with steering actuator.
%   x = [X; Y; psi; vx; vy; r; delta]
%   delta_cmd : steering command (rad), ax_cmd : longitudinal accel. command (m/s^2)
psi = x(3); vx = x(4); vy = x(5); r = x(6); delta = x(7);
[Fyf, Fyr] = axle_forces(vx, vy, r, delta, p);
vxs = max(vx, 1.0);
ddelta = min(p.delta_rate_max, max(-p.delta_rate_max, (delta_cmd - delta)/p.tau_steer));
ax = min(p.ax_max, max(p.ax_min, ax_cmd));
dx = [vx*cos(psi) - vy*sin(psi);
      vx*sin(psi) + vy*cos(psi);
      r;
      ax + vy*r - Fyf*sin(delta)/p.m;
      (Fyf*cos(delta) + Fyr)/p.m - vxs*r;
      (p.a*Fyf*cos(delta) - p.b*Fyr)/p.Iz;
      ddelta];
end
