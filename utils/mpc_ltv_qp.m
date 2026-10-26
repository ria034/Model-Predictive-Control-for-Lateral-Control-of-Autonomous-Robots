function [U, slack, pred] = mpc_ltv_qp(x0, u_prev, A_list, B_list, w_list, lo, hi, refs, Qdiag, iy, cfg, p)
%MPC_LTV_QP Build and solve the MPC quadratic program for a time-varying linear prediction model.
%   x(k+1) = A_list{k} x(k) + B_list{k} u(k) + w_list{k},  k = 1..Np
%   Decision variables z = [u(1) .. u(Nc), slack]; u is held after Nc.
%   Cost: sum (x - ref)' Q (x - ref) + r_ddelta*sum du^2 + r_delta*sum u^2
%         + w_slack*slack^2 + rho_slack*slack
%   Constraints: |u| <= delta_max, |du| <= delta_rate_max*Ts,
%                lo - slack <= x(iy) <= hi + slack, slack >= 0.
%   iy is the index of e_y in the state; refs is the e_y target over the horizon.
c = cfg.mpc; Np = c.Np; Nc = c.Nc; Ts = cfg.sim.Ts;
n = numel(x0);
Sx = zeros(n, Np);           % free response
Su = zeros(n, Nc, Np);       % sensitivity of x(k) to each u
xf = x0(:); Phi = zeros(n, Nc);
for k = 1:Np
    j = min(k, Nc);
    xf = A_list{k}*xf + w_list{k};
    Phi = A_list{k}*Phi;
    Phi(:, j) = Phi(:, j) + B_list{k};
    Sx(:, k) = xf;
    Su(:, :, k) = Phi;
end
nz = Nc + 1;
H = zeros(nz); f = zeros(nz, 1);
Q = diag(Qdiag);
for k = 1:Np
    dev = Sx(:, k); dev(iy) = dev(iy) - refs(k);
    Sk = Su(:, :, k);
    H(1:Nc, 1:Nc) = H(1:Nc, 1:Nc) + Sk'*Q*Sk;
    f(1:Nc) = f(1:Nc) + Sk'*Q*dev;
end
D = eye(Nc) - diag(ones(Nc-1, 1), -1);
H(1:Nc, 1:Nc) = H(1:Nc, 1:Nc) + c.r_ddelta*(D'*D) + c.r_delta*eye(Nc);
f(1) = f(1) - c.r_ddelta*u_prev;
H(nz, nz) = c.w_slack;
f(nz) = c.rho_slack;
H = H + 1e-8*eye(nz);

dmax = p.delta_max; dr = p.delta_rate_max*Ts;
nrow = 4*Nc + 2*Np + 1;
G = zeros(nrow, nz); h = zeros(nrow, 1); r = 0;
for i = 1:Nc
    e = zeros(1, nz); e(i) = 1;
    r = r + 1; G(r, :) = e;  h(r) = dmax;
    r = r + 1; G(r, :) = -e; h(r) = dmax;
    g = zeros(1, nz); g(i) = 1;
    if i > 1
        g(i-1) = -1;
        r = r + 1; G(r, :) = g;  h(r) = dr;
        r = r + 1; G(r, :) = -g; h(r) = dr;
    else
        r = r + 1; G(r, :) = g;  h(r) = dr + u_prev;
        r = r + 1; G(r, :) = -g; h(r) = dr - u_prev;
    end
end
for k = 1:Np
    row = zeros(1, nz); row(1:Nc) = Su(iy, :, k); row(nz) = -1;
    r = r + 1; G(r, :) = row;  h(r) = hi(k) - Sx(iy, k);
    row2 = zeros(1, nz); row2(1:Nc) = -Su(iy, :, k); row2(nz) = -1;
    r = r + 1; G(r, :) = row2; h(r) = Sx(iy, k) - lo(k);
end
r = r + 1; G(r, nz) = -1; h(r) = 0;

z = qp_interior_point(H, f, G, h);
U = z(1:Nc);
slack = z(nz);
pred = zeros(n, Np);
for k = 1:Np
    pred(:, k) = Sx(:, k) + Su(:, :, k)*U;
end
end
