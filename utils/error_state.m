function [x, s, kap, mem] = error_state(meas, path, mem)
%ERROR_STATE Lateral error state [e_y; e_y_dot; e_psi; e_psi_dot] from a (noisy) measurement.
if ~isfield(mem, 'i'), mem.i = 1; end
[i, s, ey] = project_to_path(path, meas.X, meas.Y, mem.i);
mem.i = i;
epsi = wrap_angle(meas.psi - path.psi(i));
kap = path.kappa(i);
x = [ey; meas.vy + meas.vx*epsi; epsi; meas.r - meas.vx*kap];
end
