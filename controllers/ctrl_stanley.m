function [u, mem] = ctrl_stanley(meas, path, cfg, mem)
%CTRL_STANLEY Stanley controller on the front axle, with yaw-rate damping.
p = cfg.vehicle; c = cfg.stanley;
if ~isfield(mem, 'i'), mem.i = 1; end
Xf = meas.X + p.a*cos(meas.psi);
Yf = meas.Y + p.a*sin(meas.psi);
[i, ~, ey] = project_to_path(path, Xf, Yf, mem.i);
mem.i = i;
epsi = wrap_angle(meas.psi - path.psi(i));
r_ref = meas.vx*path.kappa(i);
u = -epsi - atan(c.k*ey/(c.k_soft + meas.vx)) - c.k_yaw*(meas.r - r_ref);
end
