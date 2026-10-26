function [u, mem] = ctrl_nmpc(meas, path, cfg, mem)
%CTRL_NMPC Nonlinear MPC by real-time iteration (one SQP step per sample).
%   Each sample: roll the nonlinear model (Fiala tyres, path coordinates,
%   actuator) along the shifted previous solution, linearize it by finite
%   differences at every horizon step, and solve the resulting QP.
p = cfg.nominal; c = cfg.mpc; Np = c.Np; Nc = c.Nc; Ts = cfg.sim.Ts;
epsfd = cfg.nmpc.fd_eps; nsub = cfg.nmpc.substeps;
if ~isfield(mem, 'i'), mem.i = 1; end
[i, s, ey] = project_to_path(path, meas.X, meas.Y, mem.i);
mem.i = i;
epsi = wrap_angle(meas.psi - path.psi(i));
xi0 = [ey; epsi; meas.vy; meas.r; meas.delta];
vx = max(meas.vx, 1.0);
if cfg.robust.compensate_delay && isfield(mem, 'pending')
    for q = 1:numel(mem.pending)
        xi0 = nl_path_step(xi0, mem.pending(q), vx, path.kappa(i), p, Ts, nsub);
    end
end
% speed, curvature and corridor over the horizon
vks = zeros(Np, 1); kks = zeros(Np, 1);
lo = zeros(Np, 1); hi = zeros(Np, 1); refs = zeros(Np, 1);
ss = s;
for k = 1:Np
    j = path_index(path, ss);
    vks(k) = max(path.v(j), 1.0); kks(k) = path.kappa(j);
    ss = ss + path.v(j)*Ts;
    [lo(k), hi(k), refs(k)] = corridor_bounds(path, ss, p, c.use_obstacles);
end
% warm start: previous solution shifted by one sample
if isfield(mem, 'U'), Ug = [mem.U(2:end); mem.U(end)]; else, Ug = zeros(Nc, 1); end
if ~isfield(mem, 'u'), mem.u = 0; end
A_list = cell(Np, 1); B_list = cell(Np, 1); w_list = cell(Np, 1);
for it = 1:cfg.nmpc.iters
    xg = xi0;
    for k = 1:Np
        uk = Ug(min(k, Nc));
        fx = nl_path_step(xg, uk, vks(k), kks(k), p, Ts, nsub);
        A = zeros(5);
        for q = 1:5
            d = zeros(5, 1); d(q) = epsfd;
            A(:, q) = (nl_path_step(xg + d, uk, vks(k), kks(k), p, Ts, nsub) - fx)/epsfd;
        end
        Bv = (nl_path_step(xg, uk + epsfd, vks(k), kks(k), p, Ts, nsub) - fx)/epsfd;
        A_list{k} = A; B_list{k} = Bv; w_list{k} = fx - A*xg - Bv*uk;
        xg = fx;
    end
    [Ug, ~, pred] = mpc_ltv_qp(xi0, mem.u, A_list, B_list, w_list, lo, hi, refs, ...
                               [c.q_ey c.q_epsi 0 0 0], 1, cfg, p);
end
u = Ug(1);
mem.U = Ug;
mem.u = u;
mem.pred_ey = pred(1, :);
mem = push_pending(mem, u, cfg);
end
