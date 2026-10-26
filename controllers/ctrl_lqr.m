function [u, mem] = ctrl_lqr(meas, path, cfg, mem)
%CTRL_LQR Discrete LQR on the lateral error model, re-computed for the current speed,
%   plus the steady-state curvature feedforward (Rajamani, eq. 3.12).
p = cfg.nominal; c = cfg.lqr; Ts = cfg.sim.Ts;
[x, ~, kap, mem] = error_state(meas, path, mem);
vx = max(meas.vx, 1.0);
[A, B, E] = error_model(vx, p);
if cfg.robust.compensate_delay && isfield(mem, 'pending')
    [Ad, Bd] = c2d_zoh(A, [B E], Ts);
    for q = 1:numel(mem.pending)
        x = Ad*x + Bd(:,1)*mem.pending(q) + Bd(:,2)*vx*kap;
    end
end
[Ad, Bd] = c2d_zoh(A, B, Ts);
K = dlqr_iter(Ad, Bd, diag(c.Q), c.R);
Kv = p.m*(p.b*p.Cr - p.a*p.Cf)/(p.L*p.Cf*p.Cr);          % understeer gradient
dff = p.L*kap + Kv*vx^2*kap - K(3)*(p.b*kap - p.a*p.m*vx^2*kap/(p.Cr*p.L));
u = -K*x + dff;
u = min(p.delta_max, max(-p.delta_max, u));
mem = push_pending(mem, u, cfg);
end
