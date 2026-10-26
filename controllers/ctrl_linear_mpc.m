function [u, mem] = ctrl_linear_mpc(meas, path, cfg, mem, adaptive)
%CTRL_LINEAR_MPC Linear MPC on the lateral error model with actuator state.
%   adaptive = false : one model at the fixed design speed cfg.mpc.v_design
%   adaptive = true  : a different model at every horizon step, using the
%                      speed the profile will have there (linear time-varying MPC)
%   Both preview the path curvature and respect the obstacle corridor.
p = cfg.nominal; c = cfg.mpc; Ts = cfg.sim.Ts; Np = c.Np;
[x4, s, kap, mem] = error_state(meas, path, mem);
vx = max(meas.vx, 1.0);
A_list = cell(Np, 1); B_list = cell(Np, 1); w_list = cell(Np, 1);
lo = zeros(Np, 1); hi = zeros(Np, 1); refs = zeros(Np, 1);
ss = s;
for k = 1:Np
    i = path_index(path, ss);
    if adaptive, vk = max(path.v(i), 1.0); else, vk = c.v_design; end
    [A, B, E] = error_model_act(vk, p);
    [Ad, Bd] = c2d_zoh(A, [B E], Ts);
    A_list{k} = Ad; B_list{k} = Bd(:,1); w_list{k} = Bd(:,2)*vk*path.kappa(i);
    if adaptive, ss = ss + path.v(i)*Ts; else, ss = ss + vx*Ts; end
    [lo(k), hi(k), refs(k)] = corridor_bounds(path, ss, p, c.use_obstacles);
end
x = [x4; meas.delta];
if cfg.robust.compensate_delay && isfield(mem, 'pending')
    [A, B, E] = error_model_act(vx, p);
    [Ad, Bd] = c2d_zoh(A, [B E], Ts);
    for q = 1:numel(mem.pending)
        x = Ad*x + Bd(:,1)*mem.pending(q) + Bd(:,2)*vx*kap;
    end
end
if ~isfield(mem, 'u'), mem.u = 0; end
[U, ~, pred] = mpc_ltv_qp(x, mem.u, A_list, B_list, w_list, lo, hi, refs, ...
                          [c.q_ey 0 c.q_epsi 0 0], 1, cfg, p);
u = U(1);
mem.u = u;
mem.pred_ey = pred(1, :);
mem = push_pending(mem, u, cfg);
end
