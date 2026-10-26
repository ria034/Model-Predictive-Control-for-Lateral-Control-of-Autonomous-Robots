function [u, mem] = ctrl_pure_pursuit(meas, path, cfg, mem)
%CTRL_PURE_PURSUIT Geometric pure pursuit on the rear axle, speed-dependent look-ahead.
p = cfg.vehicle; c = cfg.pp;
if ~isfield(mem, 'i'), mem.i = 1; end
Xr = meas.X - p.b*cos(meas.psi);
Yr = meas.Y - p.b*sin(meas.psi);
[i, s] = project_to_path(path, Xr, Yr, mem.i);
mem.i = i;
Ld = min(c.Lmax, max(c.Lmin, c.k_ld*meas.vx + c.L0));
j = path_index(path, s + Ld);
gx = path.x(j); gy = path.y(j);
alpha = atan2(gy - Yr, gx - Xr) - meas.psi;
Ld_act = max(1.0, hypot(gx - Xr, gy - Yr));
u = atan(2*p.L*sin(alpha)/Ld_act);
end
